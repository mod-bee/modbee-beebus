# 02 - Getting started

This chapter builds a working two-node network from nothing. Every step says
what it does and *why it is there*, because the why is what lets you change it
later with confidence.

By the end you will have two nodes where one publishes a value and the other
receives it, knows how fresh it is, and is told if an update goes missing.

## What you need

Either of these:

- **Two microcontrollers** with RS485 transceivers wired A-to-A, B-to-B, with
  a 120 Ω terminating resistor at each end of the run, and **at least 32 KB of
  RAM** each. ESP32, ESP8266, SAMD21, STM32 — anything with a spare hardware
  serial port and room to work in. A node needs roughly 13 KB for its fixed
  tables and buffers, plus its operation queues, before any of your own data:
  the hop timing and liveness tables are indexed by node ID rather than cached,
  so every peer has its estimate ready the moment the ring needs it, and the
  receive path holds a frame queue and two frame buffers. **The 8-bit AVR
  boards — Uno, Nano, Mega — do not have the room.** The hop table can be
  [sized down](10-configuration.md) if your ring is small and your node IDs
  start at 1, and the queues if a node keeps few requests in flight.
- **One computer.** Two processes on one machine talking over UDP multicast is
  a real BeeBus network in every respect that matters, and it needs no hardware
  at all. Start here if you are just exploring.

## Step 1 — install

**Arduino.** Copy the repository into your `libraries` folder, or add it as a
ZIP library. Then, in a sketch:

```cpp
#include <BeeBusGlobal.h>
```

That one header pulls in everything in the right order. There is no second
include to remember.

**C++ / CMake.** Add the repository as a subdirectory:

```cmake
add_subdirectory(modbee-beebus)
target_link_libraries(my_app PRIVATE BeeBus::beebus)
```

Or build it standalone, which also builds the runnable examples:

```bash
cmake -S . -B build && cmake --build build -j
```

*Why one header:* the include order between the frame codec, the data maps and
the protocol core is real and easy to get wrong. `BeeBusGlobal.h` fixes it once.

## Step 2 — give each node an ID

```cpp
#define NODE_ID 1     // 1..254, unique on the segment
```

Node IDs run **1 to 254**. Zero means "nobody" in the frame header, so it is not
addressable, and 255 is reserved.

*Why you assign them by hand:* the ID is not just a name. It decides where the
node sits in the ring, because the token always goes to the next-highest ID.
Assigning IDs yourself means you know the order the token travels in, which
means you can put the slow radio node somewhere sensible and keep two nodes that
talk to each other adjacent. An auto-assigned ID would take that away for
nothing in return — you have to configure *something* per node anyway.

Two nodes with the same ID is a real fault, and BeeBus detects it: a node that
hears its own address from somebody else raises `DuplicateNodeID` and stands
down rather than corrupting the ring.

## Step 3 — open a transport and start the node

A transport is the only thing that knows what medium you are on. The protocol
core never finds out.

**Arduino, RS485:**

```cpp
BeeBusAPI beebus;

void setup() {
    Serial1.begin(115200);              // Your RS485 port
    beebus.begin(&Serial1, NODE_ID);
}
```

Optionally pass the baud rate too —
`beebus.begin(&Serial1, NODE_ID, BeeBusConfig(), 115200)`. It is the one
property of the line a node cannot measure for itself, and with it the
interframe gap and the response window are sized from the rate from the start
rather than measured first.

**Host, UDP multicast:**

```cpp
#include <transport/BeeBusHostUdpTransport.h>

BeeBusUdpConfig config;                 // Defaults to 239.192.7.126:7126
BeeBusHostUdpTransport transport(config);
BeeBusAPI beebus;

beebus.begin(&transport, NODE_ID);
```

*Why a separate transport object:* it means the ESP32 running your gateway can
carry a serial segment and a UDP segment at the same time, from two
`BeeBusAPI` instances, with no special "gateway mode" anywhere in the protocol.
It also means porting to a medium nobody has thought of yet is one small class.
See [07 - Transports](07-transports.md).

`begin()` prepares the node but does not put it on the air. That is deliberate,
and step 6 explains why.

## Step 4 — bind your data

This is the step that matters. **You bind a variable; the protocol takes it from
there.**

```cpp
float temperature = 0.0f;

beebus.addInputTopic("temperature", 1, &temperature);
```

Three arguments, in the order you will use everywhere:

| | |
| --- | --- |
| `"temperature"` | The name. This is what you use in every call afterwards. |
| `1` | The topic ID, 1–32767. This is what travels on the wire. |
| `&temperature` | The variable itself. |

*Why a name and an ID:* the name is for you and never travels in the data path —
sending 11 characters with every reading would be absurd on a radio link. The ID
is one or two bytes. You state the pairing once, at the binding, and then work
in names for the rest of the program. On a build with `BEEBUS_TOPIC_NAMES=0` the
names compile out entirely and only the IDs remain.

*Why the type is not an argument:* it is deduced from the variable at compile
time. Bind a `float` and the wire carries a float; bind a `uint16_t` and it
carries a `uint16_t`. Binding something the wire cannot express — `int`, a
struct, a pointer — fails to compile with a message telling you what is
allowed. It cannot be got wrong at runtime.

> `int` is deliberately not bindable. It is 16 bits on AVR and 32 on ESP32, so
> the same source would put different types on the wire depending on the board.
> Use `int16_t` or `int32_t` and mean it.

**Text works almost the same way.** Bind a character buffer and its capacity,
and the topic carries a message — a fault code, an identifier — with its length
travelling on the wire, so nothing at the far end needs to know how long it is:

```cpp
char fault[32] = "";
beebus.addInputTopic("fault", 20, fault, sizeof(fault));
strcpy(fault, "pump 2 overload");
```

> The buffer is yours and must outlive the binding; text arriving from a peer
> is cut to fit and always terminated, and the node allocates nothing for it.
> Arduino's `String` does not bind; pass `myString.c_str()` to any call that
> takes text.

**Input or not?**

```cpp
beebus.addInputTopic("temperature", 1, &temperature);   // Peers may only read it
beebus.addTopic("setpoint", 2, &setpoint);              // Peers may also write it
```

*Why the distinction is at the binding:* a measurement is not writable, ever, by
anybody. Saying so where the variable is bound means you cannot forget it at one
of the six call sites that might have written it. A peer that tries gets
`topic is read only` back, immediately, with a reason.

## Step 5 — bind what you want *from* other nodes

The other side of the same coin. To receive a peer's topic, you bind one of your
variables to it:

```cpp
float remoteTemp = 0.0f;

beebus.registerTopic(SENSOR_NODE, "temperature", 1, &remoteTemp);
```

Same order, with the node in front: **node, name, ID, variable.**

*Why you have to bind before you can receive:* a value arriving for a topic with
nothing bound to it is dropped. That sounds unhelpful until you consider the
alternative — the binding is what states the expected type. Without it, four
bytes of float would be stored into whatever the receiver happened to think the
topic was. The binding is the contract, and it is checked on every value.

## Step 6 — subscribe

Now say what you actually want.

```cpp
beebus.onMemberJoined([](uint8_t node) {
    if (node == SENSOR_NODE) {
        beebus.subscribe(SENSOR_NODE, "temperature", 0.5f, 60000);
    }
});
```

Two numbers, and both are yours to choose:

| | |
| --- | --- |
| `0.5f` | **Deadband**, in the value's own units. Report when it moves half a degree. `0` means report every change. |
| `60000` | **Maximum interval**, milliseconds. Report at least this often even if nothing changed. `0` promises nothing. |

*Why both:* the deadband is what keeps the network quiet — a temperature that
drifts by hundredths does not deserve airtime. The maximum interval is what
makes silence meaningful: without it, a value that stopped changing and a node
that died look identical. With it, you have a promise, and quality is judged
against that promise rather than against some constant the library made up.

*Why inside `onMemberJoined`:* subscriptions live on the publisher. A node that
restarts has lost its subscriber list, so the subscription has to be placed
again when it comes back. Doing it in the joined handler covers the first join
and every rejoin with the same three lines. (BeeBus also re-issues them for you
when a peer returns — the handler is belt and braces, and the place to put a
`fetchTopics` too.)

**Two subscribers can want different things.** One node asks for ±0.5 °C, another
for ±5 °C. Both get exactly what they asked for; the terms belong to the
subscription, not to the topic.

## Step 7 — connect, and call `loop()`

```cpp
    beebus.connect();
}

void loop() {
    beebus.loop();
    // ... your application ...
}
```

*Why `connect()` is separate from `begin()`:* everything above — bindings,
subscriptions, handlers — should be in place before the node joins a ring.
The instant it joins, peers can write to it and it can start receiving values.
`connect()` is you saying "I am ready".

*Why `loop()` must be called often:* everything happens there. Bytes are drained
from the transport, frames are assembled, the token is accepted and passed on,
subscriptions are serviced, timeouts are checked. It does not block and it does
not need a thread. Call it every pass of your main loop, and **do not put a
`delay()` in the path** — a node that stops calling `loop()` stops answering the
token, and the ring will eventually conclude it has gone.

The one bound on the work `loop()` does per call is the receive path: at most
1024 bytes are drained per call, so a flooded node still runs your application.
Anything left over is taken next time.

## Step 8 — use the data

That is the whole setup. From here, `remoteTemp` simply *is* the sensor's
temperature.

```cpp
void loop() {
    beebus.loop();

    if (millis() - lastTick < 1000) return;
    lastTick = millis();

    // The variable tracks the peer. Nothing polls it.
    Serial.print(F("remote temperature "));
    Serial.print(remoteTemp);

    // And you can ask whether you should be acting on it.
    Serial.print(F(" ["));
    Serial.print(bb_qualityName(beebus.topicQuality(SENSOR_NODE, "temperature")));
    Serial.println(F("]"));
}
```

`topicQuality()` returns one of four answers, judged against the subscription
you placed:

| | |
| --- | --- |
| `Unknown` | Nothing has ever arrived |
| `Good` | Within the interval you asked for, and the publisher is in the ring |
| `Uncertain` | Late, but less than twice the interval |
| `Bad` | Later than that, or the publisher has left the ring |

*Why this exists:* a variable holding 21.5 tells you nothing about whether 21.5
is still true. Every control decision that matters wants that second question
answered, and answering it by hand means remembering to timestamp every value
in every application. It is in the library because it should be.

## The complete sketch

Two files, differing only in the two IDs at the top.

**Sensor node (ID 1):**

```cpp
#include <BeeBusGlobal.h>

#define NODE_ID   1
#define PEER_ID   2

BeeBusAPI beebus;

float temperature = 0.0f;
int16_t command   = 0;

float readSensor() { return 21.5f; }   // Replace with your own measurement

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    beebus.begin(&Serial1, NODE_ID);

    beebus.addInputTopic("temperature", 1, &temperature);   // Read-only to peers
    beebus.addTopic("command", 10, &command);               // Peers may write it

    beebus.onTopicWritten([](uint8_t from, uint16_t topicID, const BeeBusScalar&) {
        Serial.print(F("node "));
        Serial.print(from);
        Serial.print(F(" wrote topic "));
        Serial.println(topicID);
    });

    beebus.connect();
}

void loop() {
    beebus.loop();

    static uint32_t lastTick = 0;
    if (millis() - lastTick < 500) return;
    lastTick = millis();

    // Just move the variable. The subscriber's terms decide what is sent.
    temperature = readSensor();
}
```

**Watcher node (ID 2):**

```cpp
#include <BeeBusGlobal.h>

#define NODE_ID   2
#define PEER_ID   1

BeeBusAPI beebus;

float peerTemperature = 0.0f;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    beebus.begin(&Serial1, NODE_ID);

    beebus.registerTopic(PEER_ID, "temperature", 1, &peerTemperature);

    beebus.onMemberJoined([](uint8_t node) {
        if (node == PEER_ID) {
            beebus.subscribe(PEER_ID, "temperature", 0.5f, 60000);
        }
    });

    // A lost update is reported, not merely absent.
    beebus.onTopicGap([](uint8_t node, uint16_t topicID, uint8_t missed) {
        Serial.print(F("missed "));
        Serial.print(missed);
        Serial.print(F(" updates of topic "));
        Serial.print(topicID);
        Serial.print(F(" from node "));
        Serial.println(node);
    });

    beebus.connect();
}

void loop() {
    beebus.loop();

    static uint32_t lastTick = 0;
    if (millis() - lastTick < 1000) return;
    lastTick = millis();

    Serial.print(peerTemperature);
    Serial.print(F(" ["));
    Serial.print(bb_qualityName(beebus.topicQuality(PEER_ID, "temperature")));
    Serial.println(F("]"));

    // Send a command the other way, and be told whether it landed.
    beebus.writeTopic(PEER_ID, "command", (int16_t)42, BeeBusDelivery::Confirmed);
}
```

Notice what is *not* in there. No polling loop. No message construction. No
timeout constants. No master. No addressing scheme beyond two node IDs and two
topic IDs. Two nodes power up in any order, find each other, and start moving
data.

## Trying it without hardware

```bash
cmake -S . -B build && cmake --build build -j

# Two terminals
./build/beebus_topic_node 1 2
./build/beebus_topic_node 2 1
```

Both processes join one UDP multicast group and behave exactly as two boards on
one RS485 pair would. This is the fastest way to see joining, publishing,
quality and gap reporting without touching a wire.

## What to read next

- Something not working? [12 - Best practices](12-best-practices.md) ends with
  the things that actually catch people.
- Want to understand what is happening on the wire?
  [03 - How the ring works](03-how-the-ring-works.md).
- Ready to design a real network? [13 - Applications](13-applications.md).
- Need a specific call? [08 - API reference](08-api-reference.md).
</content>
