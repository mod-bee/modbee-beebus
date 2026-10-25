# 13 - Applications

Six example networks, laid out and explained. For each one: what the problem
actually is, how to arrange the network, **why that arrangement and not another**,
and the code.

The point of this chapter is not the applications themselves. It is that in all
six, the work you do is *binding variables and stating what you want*. The
protocol handles the rest — and the interesting decisions turn out to be about
deadbands, intervals and where to put a boundary, not about messages.

---

## 1. Pump station

**The problem.** A wet well with duty and standby pumps. Level, flow and
discharge pressure. A control panel that runs the pumps, a small HMI, and a
telemetry link back to the office.

The hard requirement in every pump station: **the pumps must keep working when
the network does not.** Anything that depends on a supervisor to run the duty
cycle is a station that floods when a cable is cut.

### Layout

```
       RS485, 115200, inside the kiosk
   ┌──────┬──────────┬──────────┬──────────┐
   │      │          │          │          │
 Node 10  Node 11   Node 12   Node 20    Node 90
 level    pump 1    pump 2      HMI     telemetry
 flow     starter   starter              gateway
 pressure                                  │
                                           │ radio / 4G
                                           ▼
                                        office
```

### Why laid out this way

**The control loop lives on node 10, next to the level sensor.** It reads its own
level directly and commands the starters. The office is not in the loop and can
disappear without the station noticing.

**IDs 10, 11, 12 are adjacent.** The token goes 10 → 11 → 12, so a level change
on node 10 reaches the starters on the next two hops. That is a fraction of a
rotation, not a whole one.

**The gateway is node 90, at the end.** The slow link is on the far side of it,
so the station's own ring never waits for the radio. See
[11 - Gateways](11-gateways.md).

**Gaps in the numbering** leave room for a third pump at node 13 and a second
sensor node at 14, both landing exactly where they should in the rotation.

### The sensor node (10)

```cpp
#define NODE_ID 10

float   level      = 0.0f;    // metres
float   flow       = 0.0f;    // l/s
float   pressure   = 0.0f;    // bar
bool    dutyRun    = false;   // commanded on node 11
bool    standbyRun = false;   // commanded on node 12

void setup() {
    Serial1.begin(115200);

    BeeBusConfig config;
    config.enableFailSafe = true;
    beebus.begin(&Serial1, NODE_ID, config);

    beebus.addInputTopic("level",    1, &level);
    beebus.addInputTopic("flow",     2, &flow);
    beebus.addInputTopic("pressure", 3, &pressure);

    // The starters' run feedback, so this node can see what it commanded
    beebus.registerTopic(11, "running", 1, &dutyRun);
    beebus.registerTopic(12, "running", 1, &standbyRun);

    beebus.onMemberJoined([](uint8_t node) {
        if (node == 11) beebus.subscribe(11, "running", 0.0f, 5000);
        if (node == 12) beebus.subscribe(12, "running", 0.0f, 5000);
    });

    beebus.connect();
}

void loop() {
    beebus.loop();

    if (millis() - lastScan < 200) return;
    lastScan = millis();

    level    = readLevel();
    flow     = readFlow();
    pressure = readPressure();

    // The control loop. It runs whether or not anybody is listening.
    //
    // Sent as a coil write rather than a topic write, because this output is
    // failsafed — see the note below.
    if (level > START_LEVEL) {
        beebus.writeCoil(11, 0, true, BeeBusDelivery::Confirmed);
    } else if (level < STOP_LEVEL) {
        beebus.writeCoil(11, 0, false, BeeBusDelivery::Confirmed);
    }
}
```

### The starter node (11)

```cpp
#define NODE_ID 11

bool  run     = false;    // Written by node 10, over a coil
bool  running = false;    // Contactor auxiliary
float current = 0.0f;

void setup() {
    BeeBusConfig config;
    config.enableFailSafe = true;                    // Essential here
    beebus.begin(&Serial1, NODE_ID, config);

    // The command, as a coil, so failsafe can act on it.
    beebus.addCoil(0, &run);
    beebus.setCoilFailsafe(0, BeeBusFailsafe::ResetToValue, false);

    // The telemetry, as topics, so it carries type and quality.
    beebus.addInputTopic("running", 1, &running);
    beebus.addInputTopic("current", 2, &current);

    // Short handler: set a flag, act on it in loop().
    beebus.onCoilChanged(0, [](uint16_t, bool, uint8_t) {
        commandPending = true;
    });

    beebus.onFailsafeApplied([](BeeBusRegisterType, uint16_t addr, uint8_t lost) {
        logAlarm("stopped: controller lost", addr, lost);
    });

    beebus.connect();
}
```

> **Why the command is a coil and the telemetry is a topic.**
>
> Failsafe is a **register-layer** feature. The data map records which node last
> wrote each address, and drives that address to its safe value when that node
> leaves. A topic write stores straight into the bound variable and records no
> writer, so `setCoilFailsafe` on a topic-driven output would never fire.
>
> So: **outputs that must fail safe are commanded over coils and registers.**
> Everything else — measurements, status, anything you want typed and
> quality-stamped — is a topic.
>
> If you would rather command over topics, do the equivalent yourself:
>
> ```cpp
> if (beebus.topicQuality(10, "levelOk") != BeeBusQuality::Good) {
>     stopPump();          // Your own failsafe, on your own terms
> }
> ```
>
> That is a perfectly good design. It just has to be written down somewhere,
> and the register path already has it written down.

### Deadbands and intervals

| Topic | Deadband | Interval | Why |
| --- | --- | --- | --- |
| `level` (to starters) | `0.01` m | 2000 ms | It drives the control decision |
| `level` (to office) | `0.10` m | 300000 ms | The office wants a trend, not a controller input |
| `flow` | `0.5` l/s | 60000 ms | Below that is turbulence, not information |
| `pressure` | `0.05` bar | 60000 ms | |
| `running` | `0.0` | 5000 ms | Every change matters; the interval proves the node is alive |

**Two subscribers, two sets of terms, same topic.** The starters get level at
±1 cm; the office gets it at ±10 cm. The office's radio link carries a fraction
of the traffic and the control loop loses nothing.

### What this gains you

- **The station runs standalone.** Cut the telemetry link and nothing changes
  locally.
- **A failed controller stops the pumps**, through failsafe, and says so.
- **Adding a third pump** is a node ID, three bindings and a subscription. No
  poll list to extend, no master to reconfigure.
- **The HMI reads everything without being asked for.** It subscribes to what it
  wants to display, and quality tells it which fields to grey out.

---

## 2. Irrigation across a farm

**The problem.** Twelve valve blocks and eight soil moisture sensors spread over
several hundred hectares. Radio, because trenching is not happening. A pump shed
with a wired panel. Weeks between visits.

The hard part is not the control logic — it is that **airtime is the scarce
resource** and every design decision has to respect that.

### Layout

```
   Wired RS485 in the shed              LoRa, 900 MHz
   ┌──────┬──────────┬──────────┐    ┌──────────────────────────┐
   │      │          │          │    │      │      │      │     │
 Node 10 Node 11   Node 12    Node 90 ── 20    21     30    31 ...
 pump    filter    pressure   GATEWAY  valve valve  soil  soil
 control bank       sensor             block block  probe probe
```

### Why laid out this way

**The gateway is the whole design.** Without it, the shed's wired nodes would run
at the radio's pace: one slow hop in a ring makes every rotation slow, and every
timeout in the network derives from the rotation.

With the gateway, the shed ring rotates in milliseconds and the radio ring
rotates in seconds, **and neither knows about the other.**

**Valves are 20–29, sensors 30–39.** Function-grouped, so the token visits all
the valves and then all the sensors, and a command to several valves lands on
consecutive hops.

**Only what is wanted crosses.** The gateway subscribes to the radio nodes on
radio-appropriate terms and republishes on the shed side on shed-appropriate
terms. The shed's fast local traffic never touches the radio.

### Radio-appropriate settings

The timing needs none. The gap comes from the line, the response window widens
by however much answering out here turns out to cost, and every rotation-based
timeout is measured. What is worth stating is the listen period, because a node
on a silent bus has nothing to measure, and the retry budget, which is a policy
choice about how long to keep trying before calling a node gone.

```cpp
// On the gateway, radio side
BeeBusConfig radioConfig;
radioConfig.initialListenPeriod = 10000;   // Joining takes longer out here
radioConfig.maxRetries          = 4;
radioSide.begin(&radioTransport, 90, radioConfig);

// Each node's topics are bound with registerTopic() beforehand, so the
// names below resolve.
radioSide.onMemberJoined([](uint8_t node) {
    if (node >= 30 && node <= 39) {
        // Soil moisture moves slowly. 2% or once an hour is plenty.
        radioSide.subscribe(node, "moisture", 2.0f, 3600000);
        radioSide.subscribe(node, "battery",  0.2f, 3600000);
    }
    if (node >= 20 && node <= 29) {
        radioSide.subscribe(node, "pressure", 0.5f, 600000);
        radioSide.subscribe(node, "battery",  0.2f, 3600000);
    }
});
```

Everything else measures itself, which on a radio is the entire point — a hop out
there might be 800 ms, and no configuration file was going to guess that.

### The valve node

```cpp
#define NODE_ID 20

bool  open      = false;    // Commanded from the shed
float pressure  = 0.0f;
float battery   = 0.0f;

void setup() {
    BeeBusConfig config;
    config.enableFailSafe = true;
    config.maxRetries     = 4;
    beebus.begin(&radio, NODE_ID, config);

    // The command is a coil, so failsafe applies: a valve left open unattended
    // empties the tank. See the note in section 1.
    beebus.addCoil(0, &open);
    beebus.setCoilFailsafe(0, BeeBusFailsafe::ResetToValue, false);

    // Telemetry as topics, so it carries type, quality and history.
    beebus.addInputTopic("pressure", 2, &pressure);
    beebus.addInputTopic("battery",  3, &battery);

    // Radio fades. Hold the readings and deliver them with their real ages.
    beebus.bufferTopic("pressure", 30);
    beebus.bufferTopic("battery",  10);

    beebus.connect();
}
```

### Why buffering matters here

A fade of ten minutes with a 600-second interval means the shed sees `Bad` and
knows the node is unreachable — good. But the pressure readings taken during
that fade are exactly the ones you want when working out why a line burst.

`bufferTopic` holds them. When the link comes back they are delivered **in
order, with their true ages attached** — so a reading taken eight minutes ago
arrives stamped as eight minutes old, not as current. Nothing in the shed
mistakes recovered history for live data.

And if the fade outlasts the buffer, the overwritten samples are **counted and
spent as sequence numbers**, so `onTopicGap` fires with the number lost (exact
up to 126; a larger loss reports as 126). Silence is never ambiguous.

### The airtime arithmetic

| | |
| --- | --- |
| Topic IDs kept under 128 | 1 byte instead of 2, on every single PDU |
| A `PUBLISH` of a float | 8 bytes of PDU + 3 of section overhead |
| 20 radio nodes reporting hourly | ~220 bytes an hour, total |
| `fetchTopics` with names, 20 nodes | Minutes of airtime — **do it once at commissioning, or bind by ID and skip it** |
| Confirmed telemetry | Doubles your traffic for nothing. Use it for valve commands only. |

**The 2% deadband on moisture is the biggest lever in the whole design.** Soil
moisture that drifts by hundredths would otherwise publish continuously and
achieve nothing.

### What this gains you

- **Valves and sensors are the same protocol as the shed.** One library, one
  mental model, one set of tools.
- **A radio node that has been asleep or unreachable rejoins by itself** and its
  subscriptions are re-issued automatically.
- **Battery voltage is a topic like any other**, with quality — so a node whose
  battery reading has gone `Bad` is visibly different from one reading 3.2 V.
- **Adding a valve block** is one node ID and three bindings.

---

## 3. Environmental monitoring — river and rainfall

**The problem.** Battery-powered nodes along a catchment: water level, rainfall,
conductivity, temperature. Solar with a small panel. Some sites are in a gully
with marginal radio. Data must not be lost, because a missed peak during a flood
event is the one reading anyone will ever ask about.

This is the case store-and-forward exists for.

### Layout

```
    Node 30      Node 31       Node 32          Node 90
   upstream    mid-catchment  downstream   ──   base station
   level       rainfall       level             gateway ──► MQTT
   temp        temp           conductivity
```

Each node wakes, samples, joins, delivers, and goes back to sleep — or stays up
and simply rides through the fades.

### The sensor node

```cpp
#define NODE_ID 30

float level        = 0.0f;
float temperature  = 0.0f;
float conductivity = 0.0f;
float battery      = 0.0f;

void setup() {
    BeeBusConfig config;
    config.initialListenPeriod = 10000;
    config.maxRetries          = 4;
    beebus.begin(&radio, NODE_ID, config);

    beebus.addInputTopic("level",        1, &level);
    beebus.addInputTopic("temperature",  2, &temperature);
    beebus.addInputTopic("conductivity", 3, &conductivity);
    beebus.addInputTopic("battery",      4, &battery);

    // Every sample is recorded, network up or down. Nothing is lost silently.
    beebus.bufferTopic("level",        120);   // 2 hours at one a minute
    beebus.bufferTopic("temperature",   60);
    beebus.bufferTopic("conductivity",  60);
    beebus.bufferTopic("battery",       24);

    beebus.connect();
}

void loop() {
    beebus.loop();

    if (millis() - lastSample < 60000) return;
    lastSample = millis();

    // Just move the variables. The scan records every change into the ring,
    // and the ring is emptied when there is somewhere to send.
    level        = readLevel();
    temperature  = readTemperature();
    conductivity = readConductivity();
    battery      = readBattery();
}
```

### Why this works

**Every publish of a buffered topic goes through the ring — network up or down.**
There is one path and no fast case, so a live reading can never arrive ahead of
the history behind it. (If it could, the receiver would see sequence *n+40* then
*n*, *n+1*, … read each as a backward jump, and discard the whole history.)

**Ages travel with the data.** A sample held for 90 minutes arrives carrying a
90-minute age, and the receiver **back-dates** its arrival time. Quality,
freshness and newest-wins all then work unchanged — and the variable ends up
holding the *newest* sample even though the flush arrived oldest first.

**Every entry reaches `onTopicChanged` in arrival order**, so a logger gets the
complete series in the right order while the variable holds the truth.

**The flush waits 4 rotations after joining** (by default; see
`rotationsFlushSettle` in [10 - Configuration](10-configuration.md)). A node that has just woken is
joining a ring whose membership is still settling, and a history poured into
that is a history poured away.

**Four samples per token, and never more than half the queue**, so recovering
two hours of history does not lock out live traffic.

### At the base station

```cpp
beebus.onTopicChanged([](uint8_t node, uint16_t topicID, const BeeBusScalar& value) {
    // Fires for every sample, including recovered history, in order.
    logToDatabase(node, topicID, bb_scalarToFloat(value),
                  beebus.topicAge(node, topicID));    // Real age, back-dated
});

beebus.onTopicGap([](uint8_t node, uint16_t topicID, uint8_t missed) {
    // Samples went missing. `missed` says how many.
    logGap(node, topicID, missed);
});
```

That gap handler is the difference between a dataset you can defend and one you
cannot. **A gap that is recorded is a known unknown; a gap that is invisible is a
fabricated dataset.**

### Sizing the buffers

Buffer depth × sample interval = how long an outage you can ride through.

| | |
| --- | --- |
| 120 samples at 1/minute | 2 hours |
| 60 samples at one per 5 minutes | 5 hours |
| 127 (the cap) at 1/hour | 5 days |

The cap is 127 because a one-byte sequence with a "delta over 127 means stale"
rule cannot express a longer flush. A deeper buffer would have its tail silently
discarded by the receiver, which is worse than not offering it.

Each sample of a numeric topic costs 16 bytes. The four buffers in the sensor
node above hold 264 samples between them: a little over 4 KB.

---

## 4. Greenhouse, with a dashboard

**The problem.** Thirty small nodes in a growing operation — zone temperature and
humidity, vent actuators, irrigation solenoids, CO₂, light. A grower who wants a
dashboard on a phone, and a horticultural consultant who wants the data.

Two quite different audiences: the control system, and the humans.

### Layout

```
   RS485 backbone through the houses                    WiFi
   ┌──────┬───────┬───────┬───── ... ─────┬─────────┐
   │      │       │       │               │         │
 Node 10 Node 11 Node 12 ...           Node 40   Node 90 ──► MQTT broker
 zone 1  zone 2  zone 3                climate    gateway     └► dashboard
 sensors sensors sensors               controller             └► logger
                                                              └► consultant
```

### Why laid out this way

**Sensors, actuators and control are all on one ring**, because they are the same
medium and the same speed, and they all need to talk to each other. Splitting
them would add a failure point for nothing.

**The MQTT gateway is a leaf.** It is node 90, it subscribes to what the outside
world wants, and if the WiFi or the broker dies, the greenhouse does not notice.

**MQTT is for supervision, never for control.** The vents are driven by node 40
over BeeBus, with confirmed writes and failsafe. The dashboard watches, and can
send setpoints — but it cannot be in the control path, because a broker on a
phone network is not something a crop should depend on.

### Commissioning by discovery

Thirty nodes is where discovery starts to pay for itself:

```cpp
beebus.onMemberJoined([](uint8_t node) {
    beebus.fetchTopics(node, true);       // Walk the whole list, with names
});

beebus.onTopicList([](uint8_t node, const BeeBusTopicEntry& entry) {
    Serial.printf("node %u topic %u  %-14s %s\n",
                  node, entry.topicID, entry.name, bb_topicTypeName(entry.type));
});
```

Then subscribe by pattern rather than by list:

```cpp
// Once the fetch has completed
beebus.subscribeAll(node, "temp*",     0.2f, 300000);
beebus.subscribeAll(node, "humidity*", 1.0f, 300000);
```

Globs, not MQTT wildcards: `*` is any run of characters, `?` is exactly one. Flat
names, so there are no levels and no level wildcards.

`subscribeAll` requires a completed `fetchTopics` and returns `NotMapped` without
one — and a match still needs a variable bound to land in, so unregistered
matches are passed over rather than half subscribed.

On a wired backbone the naming traffic is free. On a radio it would not be, which
is why `fetchTopics(node, false)` exists.

### The MQTT bridge

See [11 - Gateways](11-gateways.md) and
`examples/arduino/Topics_MQTT_Gateway/` for the working sketch. The two rules
that matter:

**Publish from `onTopicChanged`, not from a timer.** The deadband and interval on
the BeeBus subscription already decided what deserves to travel. Re-sampling on a
timer throws that away and floods the broker with duplicates.

**Publish quality alongside every value.**

```
greenhouse/node10/temperature          21.4
greenhouse/node10/temperature/quality  Good
```

A number on an MQTT broker has no age and no provenance, and *retained messages
make this worse* — a stale retained value looks exactly like a fresh one to every
subscriber that connects afterwards. The quality topic is what turns "the last
number we saw" into "the last number we saw, and whether it is still true".

### Downlink

```
greenhouse/node40/tempSetpoint/set  →  writeTopic(40, "tempSetpoint", 21.5f, Confirmed)
greenhouse/node40/tempSetpoint/result  ←  "Immediate"
```

**Confirmed, always, for anything arriving from a broker.** MQTT's QoS ends at
the gateway; it says nothing about whether the value reached node 40. Publishing
the result back closes a loop that MQTT alone leaves open — the grower who
changes a setpoint on a phone can see that it landed.

### What this gains you

- **The growing operation runs without the internet.** The dashboard is a view,
  not a dependency.
- **Adding a zone is one node**, and the gateway discovers its topics by name.
- **The consultant gets clean, quality-stamped data** without a database
  integration project.

---

## 5. Machine skid with an existing SCADA

**The problem.** A packaged skid — pumps, valves, instruments, a local
controller — being built into a plant that already has a SCADA system speaking
Modbus TCP. The skid's internals should be modern; the plant's interface must be
what the plant already understands.

### Layout

```
   RS485 inside the skid                              Ethernet
   ┌──────┬──────────┬──────────┬──────────┐
   │      │          │          │          │
 Node 10 Node 11   Node 12    Node 20    Node 90 ──► Modbus TCP ──► plant SCADA
 inlet   outlet    dosing     skid         gateway
 sensors sensors   pump       controller
```

### The trick: two views of one variable

**The same C++ variable can be a topic and a register.**

```cpp
// On the gateway, node 90
float   inletPressure       = 0.0f;   // Native: typed, with quality
int16_t inletPressureScaled = 0;      // For the SCADA: 0.01 bar per count
int16_t inletQualityReg     = 0;      // For the SCADA: the quality, as a number

void setup() {
    beebus.begin(&Serial1, 90, config);

    // The skid's own view
    beebus.registerTopic(10, "pressure", 1, &inletPressure);

    // The plant's view — ordinary Modbus registers
    beebus.addIreg(30, &inletPressureScaled);
    beebus.addIreg(31, &inletQualityReg);

    beebus.onMemberJoined([](uint8_t node) {
        if (node == 10) beebus.subscribe(10, "pressure", 0.02f, 30000);
    });

    beebus.connect();
}

void loop() {
    beebus.loop();

    inletPressureScaled = (int16_t)(inletPressure * 100.0f);
    inletQualityReg     = (int16_t)beebus.topicQuality(10, "pressure");

    modbusTcpSlave.task();      // Any Modbus TCP slave library
}
```

The SCADA sees an ordinary Modbus device at a stated address map. The skid's own
nodes see typed topics with quality. **Nothing translates between them, because
there is nothing to translate** — they are the same variables.

### Two things worth doing

**Give the SCADA a quality register.** A SCADA reading register 30 has no idea
whether node 10 is alive. Register 31 saying `Bad` is the difference between an
operator trusting a frozen number and an operator seeing an alarm.

**Write the scaling down.** A register is sixteen anonymous bits — exactly the
problem topics exist to solve, and here you have stepped back into it
deliberately, at the boundary, where the plant needs it. Document it in the
handover pack:

```
30  Inlet pressure       int16, 0.01 bar/count, 0..1000 = 0..10 bar
31  Inlet quality        0 Unknown  1 Good  2 Uncertain  3 Bad
40  Dosing setpoint      int16, 0.1 l/h/count   [read/write]
```

### Strings for fault messages

```cpp
char faultMessage[64];
beebus.addInputString(500, faultMessage, sizeof(faultMessage));
beebus.setInputString(500, "dosing pump overload");
```

Text packed two characters to a register, big-endian, NUL terminated — so it
arrives as ordinary input registers 500–531 and the SCADA needs to know nothing
about BeeBus.

Topics carry text too — `beebus.addInputTopic("fault", 20, faultText,
sizeof(faultText))` with a `char` buffer, and the length travels on the wire so
nothing has to know the size. Use that between your own nodes. The register-backed string above is for
the **SCADA**, which speaks registers and nothing else.

### What this gains you

- **The skid is modern inside and conventional outside.** No compromise on
  either.
- **Commissioning against the plant is a register list**, which is what the
  plant's integrator wants anyway.
- **Adding an instrument to the skid** does not touch the plant interface unless
  you want it to.

---

## 6. Distributed plant with interlocks between panels

**The problem.** Three panels in different parts of a site — a raw water intake,
a treatment building, a reservoir. Each has its own local control. They must
coordinate: the intake must not run when the reservoir is full; treatment must
not dose when there is no flow.

Classically this is a wire between panels, or a PLC that owns everything.

> **A note before the pattern.** BeeBus is not a functional safety protocol.
> There is no SIL rating, no black-channel, no authentication. Anything that
> protects people belongs in a hardwired safety circuit or a rated safety system.
> What follows is about *process* interlocks and permissives — the ones that
> protect equipment and process, and which today are usually a spare pair in a
> multicore.

### Layout

```
      Reservoir panel        Treatment panel         Intake panel
         Node 10                Node 20                Node 30
            │                      │                      │
            └──────────────────────┴──────────────────────┘
                        RS485 backbone, 19200
                     (long run, so slower baud)
```

### Why peer-to-peer wins here

**Each panel publishes its state. Each panel subscribes to what it needs.** There
is no central controller, so there is no node whose failure stops the site.

```cpp
// Reservoir panel, node 10
float reservoirLevel = 0.0f;
bool  reservoirFull  = false;

beebus.addInputTopic("level", 1, &reservoirLevel);
beebus.addInputTopic("full",  2, &reservoirFull);
```

```cpp
// Intake panel, node 30
float remoteLevel = 0.0f;
bool  remoteFull  = false;

beebus.registerTopic(10, "level", 1, &remoteLevel);
beebus.registerTopic(10, "full",  2, &remoteFull);

beebus.onMemberJoined([](uint8_t node) {
    if (node == 10) {
        beebus.subscribe(10, "level", 0.05f, 5000);   // 5 cm, or every 5 s
        beebus.subscribe(10, "full",  0.0f,  5000);   // Every change, or every 5 s
    }
});

bool intakePermissive() {
    // Never act on a value you cannot trust.
    if (beebus.topicQuality(10, "full") != BeeBusQuality::Good) {
        return false;                    // Fail closed
    }
    return !remoteFull;
}
```

### The three lines that make it safe to do at all

```cpp
if (beebus.topicQuality(10, "full") != BeeBusQuality::Good) return false;
```

**This is the whole design.** An interlock over a network is only as good as its
ability to notice that the network is gone.

- The subscription promised a report **at least every 5 seconds**.
- Quality goes `Uncertain` past 5 s and `Bad` past 10 s.
- The permissive fails closed on anything other than `Good`.

So a cut cable, a dead panel, or a node stuck in a blocking loop all produce the
same, correct, conservative answer within a bounded time that **you stated when
you subscribed** — not one the library picked.

Compare with the naive version of this over any publish-on-change protocol
without a maximum interval: `remoteFull` stays `false`, for ever, and the intake
runs into a full reservoir. The interval is not a nicety; it is the mechanism.

### The rest of the pattern

**Slower baud on a long run.** 19200 over a few hundred metres between buildings
is far more robust than 115200, and a three-node ring at 19200 still rotates in
tens of milliseconds — an order of magnitude faster than the interlock needs.

**Confirmed writes for commands between panels**, over coils, so failsafe
applies (see the note in section 1):

```cpp
// On the intake panel, commanding the treatment panel
beebus.writeCoil(20, DOSING_ENABLE, true, BeeBusDelivery::Confirmed);

beebus.onWriteComplete([](uint8_t node, uint16_t address, BeeBusResult result) {
    if (result != BeeBusResult::Immediate) {
        raiseAlarm("interpanel command not delivered", node, bb_resultName(result));
    }
});
```

**Failsafe on every output driven from another panel:**

```cpp
// On the treatment panel
beebus.addCoil(DOSING_ENABLE, &dosingEnable);
beebus.setCoilFailsafe(DOSING_ENABLE, BeeBusFailsafe::ResetToValue, false);

beebus.onFailsafeApplied([](BeeBusRegisterType, uint16_t addr, uint8_t lost) {
    raiseAlarm("failsafe applied, controlling panel lost", addr, lost);
});
```

### What this gains you

- **No panel is a single point of failure** for the others' local control.
- **The interlock has a stated, enforced freshness requirement** rather than an
  implicit one.
- **Adding a fourth panel** is a node ID and a set of subscriptions.
- **One pair of wires** replaces a multicore of hardwired signals — and unlike
  the multicore, it tells you when it has failed.

---

# Patterns worth stealing

Whatever you are building, these recur.

### Keep the control loop next to the measurement

Everything else is supervision. A design where the loop closes across the network
is a design that fails when the network does — and the network *will* fail, on a
Sunday.

### Put a gateway where the media change

A slow segment and a fast segment on one ring means everybody runs at the slow
segment's pace, because every timeout derives from the rotation. Separated by a
gateway, each runs at its own, and only what the slow side asked for crosses.

### One topic, several subscribers, several sets of terms

The local controller asks for ±0.01 and 2 seconds. The office over a radio asks
for ±0.5 and ten minutes. Both get exactly what they asked for, and neither
compromises for the other. **This is the feature that makes mixed-speed networks
practical.**

### Always state a maximum interval on a measurement

It is what makes silence meaningful, and it is the number quality is judged
against. Without it, a frozen sensor and a healthy one look the same.

### Always check quality before acting

Three lines, and it is the difference between a system that degrades safely and
one that acts confidently on stale data.

### Relay quality across every gateway

The far side sees the gateway as the publisher, and the gateway is alive whether
or not the source is. Three lines again, and without them your gateway quietly
lies to everything downstream.

### Buffer what you cannot afford to lose

Any node whose link can be interrupted, whose data has value after the fact.
Ages travel with the samples, and losses are counted and reported rather than
silently absorbed.

### Confirmed for commands, unconfirmed for telemetry

The next reading is along shortly. The setpoint is not.

### Command failsafed outputs over registers, measure over topics

Failsafe records the last writer, and only register-path writes are recorded. So
outputs that must drop to a safe state go over coils and registers; measurements,
status and anything you want typed and quality-stamped are topics. Both live on
the same wire, in the same frame, on the same variables.

### Leave gaps in every numbering scheme

Node IDs, topic IDs, register addresses. It costs nothing now and saves a
renumbering exercise later — and node IDs are not just names, they decide the
token order.

---

# The thing all six have in common

Look back at the code. In every one of these networks, what you actually wrote
was:

```cpp
beebus.addInputTopic("level", 1, &level);          // Here is my data
beebus.subscribe(10, "level", 0.05f, 5000);        // Here is what I want
if (beebus.topicQuality(10, "level") == BeeBusQuality::Good) ...  // Is it trustworthy?
```

No polling loops. No message construction. No master election. No timeout
constants. No addressing scheme beyond node IDs and topic IDs. No broker to
install, keep running, and explain to whoever inherits the site.

**You focus on the data. Not the protocol itself.**
</content>
