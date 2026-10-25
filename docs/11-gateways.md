# 11 - Gateways

Joining two BeeBus segments, and bridging a BeeBus network to the outside world.

## The pattern

A gateway is a node that runs **two `BeeBusAPI` instances**, one per transport,
and calls `loop()` on both.

```cpp
BeeBusAPI serialSide, udpSide;

serialSide.begin(&Serial1, NODE_ID);
udpSide.begin(&udpTransport, NODE_ID);

void loop() {
    serialSide.loop();
    udpSide.loop();
}
```

There is no "gateway mode" in the protocol, no special frame, and no forwarding
table. The two segments are **separate BeeBus networks**: separate token rings,
separate address spaces, separate measured timing. This node simply happens to
be a member of both.

**The node may carry the same ID on both**, because neither ring can hear the
other.

## How a value crosses

**By being the same variable.**

```cpp
float analogValue = 0.0f;

// Registered on the serial side, so the field node's publishes land here
serialSide.registerTopic(FIELD_NODE, "analogValue", 1, &analogValue);

// Owned on the UDP side, so nodes there can subscribe to the same variable
udpSide.addInputTopic("analogValue", 1, &analogValue);
```

That is the whole mechanism, and **neither direction needs forwarding code**.
The publish scan on the UDP side sees the variable change and reports it to
whoever subscribed there, on their terms. A value arriving on the serial side
writes the variable; the UDP side notices.

Downstream is the mirror image:

```cpp
int16_t command = 0;

udpSide.addTopic("command", 10, &command);       // Written from the UDP side
serialSide.addTopic("command", 10, &command);    // Subscribed from the serial side
```

A write on the UDP side stores into `command`; the serial side's scan sees it
change and publishes it to the field node that subscribed.

Registers cross exactly the same way:

```cpp
int16_t sharedRegister = 0;
serialSide.addHreg(200, &sharedRegister);
udpSide.addHreg(200, &sharedRegister);
```

## What does *not* cross, and what to do about it

### Quality

This is the important one, and it is easy to miss.

Segment B sees the **gateway** as the publisher. The gateway is alive whether or
not the field node is. So a source that has died on segment A reports as `Good`
on segment B — correct about the gateway, and quite wrong about the value.

Relay it as a topic of its own:

```cpp
uint8_t analogQuality = (uint8_t)BeeBusQuality::Unknown;
udpSide.addInputTopic("analogQuality", 2, &analogQuality);

void loop() {
    serialSide.loop();
    udpSide.loop();

    analogQuality = (uint8_t)serialSide.topicQuality(FIELD_NODE, "analogValue");
}
```

Now the supervisor on segment B can tell a live value from a stale one, and the
alarm it raises names the right fault.

**Do this for every value that crosses a gateway and matters.** It is three
lines and it is the difference between a network that tells you the truth and
one that quietly lies to you.

### Membership

Segment A cannot see segment B's nodes and vice versa. `isMember()`,
`onMemberJoined` and `getMemberCount()` are per instance.

That is a feature, not a limitation: a node failing on one segment does not
disturb the other segment's ring at all.

### Sequence numbers and gaps

Each hop has its own sequence stream. A gap on segment A is reported at the
gateway, not at the far end of segment B — the gateway republishes on its own
stream, which is unbroken.

If a downstream node needs to know about upstream losses, relay that too: count
gaps in the gateway's `onTopicGap` and publish the count as a topic.

### Store-and-forward buffers

Per instance. A gateway that must not lose data across an outage on either side
needs a buffered topic on that side.

## Worked example: RS485 to UDP

The complete pattern, from `examples/arduino/Topics_Gateway/`:

```cpp
#include <BeeBusGlobal.h>
#include <transport/BeeBusArduinoUdpTransport.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define NODE_ID     10   // The same ID on both segments: separate rings
#define FIELD_NODE  1    // On the serial segment

WiFiUDP udp;
BeeBusUdpConfig udpConfig;
BeeBusArduinoUdpTransport udpTransport(&udp, udpConfig);

BeeBusAPI serialSide, udpSide;

float   analogValue    = 0.0f;
uint8_t analogQuality  = (uint8_t)BeeBusQuality::Unknown;
int16_t command        = 0;
int16_t sharedRegister = 0;

void setup() {
    Serial1.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) delay(250);

    IPAddress group;
    group.fromString(udpConfig.groupAddress);
    udp.beginMulticast(group, udpConfig.port);

    serialSide.begin(&Serial1, NODE_ID);
    udpSide.begin(&udpTransport, NODE_ID);

    // Upstream
    serialSide.registerTopic(FIELD_NODE, "analogValue", 1, &analogValue);
    udpSide.addInputTopic("analogValue", 1, &analogValue);
    udpSide.addInputTopic("analogQuality", 2, &analogQuality);

    // Downstream
    udpSide.addTopic("command", 10, &command);
    serialSide.addTopic("command", 10, &command);

    // A register, for a stock Modbus tool on either side
    serialSide.addHreg(200, &sharedRegister);
    udpSide.addHreg(200, &sharedRegister);

    // Placed when the field node joins, and again if it leaves and comes back
    serialSide.onMemberJoined([](uint8_t joined) {
        if (joined == FIELD_NODE) {
            serialSide.subscribe(FIELD_NODE, "analogValue", 0.1f, 10000);
        }
    });

    serialSide.connect();
    udpSide.connect();
}

void loop() {
    serialSide.loop();
    udpSide.loop();

    analogQuality = (uint8_t)serialSide.topicQuality(FIELD_NODE, "analogValue");
}
```

Two instances means two complete nodes' worth of RAM — two sets of frame buffers,
timing tables and operation queues, roughly **50 KB** together at the board
defaults. Comfortable on an ESP32; on a smaller board, trim the queues and the
hop table first (see [10 - Configuration](10-configuration.md#memory-and-trimming-it)).

There is a C++ equivalent in `examples/cpp/gateway_node/`.

## Rate matching

A gateway is the natural place to change the rate at which data flows, because
the two sides are genuinely different networks.

```cpp
// Fast on the wire — this is cheap
serialSide.subscribe(FIELD_NODE, "analogValue", 0.1f, 5000);
```

Then let the far side subscribe on its own, looser terms. A supervisor over a
radio subscribing at ±2.0 with a ten-minute interval gets exactly that, and the
gateway's fast local updates cost the radio nothing.

**This is the single best reason to put a gateway in.** A slow segment and a fast
segment on one ring means everybody runs at the slow segment's pace. Separated by
a gateway, each runs at its own, and only what the slow side asked for crosses.

## Bridging to MQTT

The same pattern, with an MQTT client on one side instead of a second BeeBus
instance.

`examples/arduino/Topics_MQTT_Gateway/` is a complete working sketch, using the
PubSubClient library. The snippets below are trimmed from it: helpers such as
`nameOf()`, `parseSetTopic()` and `resultTopic()` stand for the sketch's own
string handling, not BeeBus calls. The shape:

```
   RS485 field nodes                gateway                 MQTT broker
   ─────────────────                ───────                 ───────────
   pressure  ──subscribe──►  onTopicChanged  ──publish──►  site/node1/pressure
   level     ──subscribe──►                  ──publish──►  site/node1/pressure/quality
                                                                │
   setpoint  ◄──writeTopic──  MQTT callback  ◄──subscribe──  site/node1/setpoint/set
```

### Uplink — BeeBus to the broker

Subscribe on the BeeBus side, publish from `onTopicChanged`:

```cpp
beebus.onTopicChanged([](uint8_t node, uint16_t topicID, const BeeBusScalar& value) {
    char topic[64], payload[24];
    snprintf(topic, sizeof(topic), "%snode%u/%s", MQTT_BASE, node, nameOf(node, topicID));
    snprintf(payload, sizeof(payload), "%.3f", (double)bb_scalarToFloat(value));
    mqtt.publish(topic, payload, true);        // retained
});
```

**Publish from the change event, not from a timer.** The deadband and interval
you set on the BeeBus subscription have already decided what deserves to travel;
re-sampling on a timer throws that away and floods the broker with duplicates.

### Publish the quality too

```cpp
snprintf(topic, sizeof(topic), "%snode%u/%s/quality", MQTT_BASE, node, name);
mqtt.publish(topic, bb_qualityName(beebus.topicQuality(node, topicID)), true);
```

A number on an MQTT broker has no age and no provenance. Retained messages make
this worse, not better — a stale retained value looks exactly like a fresh one to
every subscriber that connects afterwards.

**Publishing quality alongside every value is what makes an MQTT bridge
trustworthy.** It is one extra topic and it turns "the last number we saw" into
"the last number we saw, and whether it is still true".

### Downlink — the broker to BeeBus

Subscribe to a command path and write into the BeeBus topic:

```cpp
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    uint8_t node; char name[BEEBUS_TOPIC_NAME_MAX];
    if (!parseSetTopic(topic, node, name, sizeof(name))) return;

    char text[24];
    size_t n = length < sizeof(text) - 1 ? length : sizeof(text) - 1;
    memcpy(text, payload, n);
    text[n] = '\0';

    beebus.writeTopic(node, name, (float)atof(text), BeeBusDelivery::Confirmed);
}
```

**Use `Confirmed` for anything arriving from the broker.** MQTT QoS ends at the
gateway; it says nothing about whether the value reached the field node. A
confirmed write does, and you can publish the outcome back:

```cpp
beebus.onWriteComplete([](uint8_t node, uint16_t topicID, BeeBusResult result) {
    mqtt.publish(resultTopic(node, topicID), bb_resultName(result));
});
```

Now an operator who sets a value from a dashboard can see whether it actually
landed. That closes a loop that MQTT alone leaves open.

### Last will

```cpp
mqtt.connect(clientID, user, pass, statusTopic, 0, true, "offline");
mqtt.publish(statusTopic, "online", true);
```

The broker publishes `offline` on your behalf if the gateway drops off. Without
it, a gateway that loses power leaves a dashboard full of confidently displayed
values that stopped updating an hour ago.

### Reconnection

When MQTT comes back, republish. `onTopicChanged` will not fire again for values
that have not moved, so a naive reconnect leaves the broker with nothing until
each value next changes:

```cpp
void onMqttReconnected() {
    for (each relayed topic) publishValueAndQuality(...);
}
```

Retained messages cover new *subscribers*, not a reconnecting publisher.

### What not to do

- **Do not bridge every topic by default.** MQTT makes it easy to publish
  everything, and then every dashboard, every logger and every integration is
  subscribed to a firehose. Bridge what is wanted outside the plant.
- **Do not let MQTT drive control loops.** The bridge is for supervision,
  logging and operator input. Interlocks and closed-loop control belong on the
  BeeBus segment, where delivery is confirmed and quality is knowable.
- **Do not put the broker's address on a node that has to keep running without
  it.** A gateway blocked on a TCP connect is a gateway not calling `loop()`.

## Bridging to Modbus TCP

BeeBus's register model exists for this.

Bind the values you want visible as holding or input registers on the gateway,
run any Modbus TCP slave library alongside, and point it at the same variables.

```cpp
float pressure = 0.0f;
int16_t pressureScaled = 0;

beebus.registerTopic(PUMP, "pressure", 1, &pressure);   // Native, typed
beebus.addIreg(30, &pressureScaled);                    // Visible to SCADA

void loop() {
    beebus.loop();
    pressureScaled = (int16_t)(pressure * 10.0f);       // 0.1 bar per count
    modbusTcpSlave.task();
}
```

The SCADA sees an ordinary Modbus device. Your nodes see typed topics with
quality. Nothing translates between them because there is nothing to translate —
they are the same variables.

Two things worth doing:

- **Publish quality as a register too.** A SCADA reading register 30 has no idea
  whether the pump is alive. Give it a status register that says so.
- **Scale explicitly, and document the scaling.** A register is sixteen anonymous
  bits; this is exactly the problem topics exist to solve, and you have just
  stepped back into it deliberately. Write the scale factor down.

## When *not* to use a gateway

A gateway is a node, so it is a thing that can fail, and it partitions your
network by design.

If two segments are the same medium, the same speed, and short enough to be one
cable, **make them one segment.** One ring is simpler, has no single point of
failure between halves, and lets any node talk to any other directly.

Reach for a gateway when there is a real boundary:

- Different media (RS485 and UDP, wired and radio)
- Very different speeds, where one segment would otherwise pace the other
- An administrative or physical boundary — separate buildings, separate panels,
  separate ownership
- A need to limit what crosses, for bandwidth or for safety

## Redundant gateways

Two gateways between the same pair of segments is possible and needs thought.

They must carry **different node IDs on each segment** — the "same ID on both
sides" trick works because no ring hears the other, which stops being true when
two gateways bridge the same pair.

Both will relay, so downstream nodes see two publishers for the same value.
Options:

- **Have each gateway relay a distinct set of topics.** Simple, and gives you
  load sharing rather than redundancy.
- **Have the standby watch the primary's heartbeat topic** and only bind its
  relay topics when the primary goes quiet. More code, real failover.

There is no automatic election, and adding one would mean deciding, in the
library, what "primary" means for your plant. That decision is better made where
the plant is understood.
</content>
