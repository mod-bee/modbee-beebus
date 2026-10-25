# BeeBus

**A peer-to-peer protocol for devices that need to talk to each other.**

No master. No broker. No polling. You bind a variable, somebody else asks for
it, and it arrives — on the terms they asked for.

Runs on Arduino and as a plain C++ library on Linux, macOS and Windows. Over
RS485, over radio, over UDP.

---

## If you know MQTT and Modbus, you already know BeeBus

BeeBus carries **two data models over the same wire**, and you can use both at
once on the same node — even on the same variable.

### Topics — MQTT, without the broker

The preferred way to move data, and the one to reach for first.

```cpp
beebus.addInputTopic("temperature", 1, &temperature);   // I publish this
beebus.subscribe(1, "temperature", 0.5f, 60000);        // Tell me when it moves
```

If you have used MQTT, this is the same idea: named values, publish and
subscribe, per-subscriber terms. What is different:

| MQTT | BeeBus topics |
| --- | --- |
| A broker every message passes through | **No broker.** Nodes talk to each other directly |
| Subscribe to a topic, get whatever arrives | Subscribe **on your terms** — a deadband and a maximum interval you choose, per subscriber |
| Payload is bytes; both ends must agree what they mean | Values carry their **type**, checked at the far end |
| A value that stops arriving looks the same as one that has not changed | `topicQuality()` tells you `Good` / `Uncertain` / `Bad` against the interval *you* asked for |
| QoS 1/2 need a session and a broker to track them | Gap detection is built in — every publish is sequenced, so a loss is *reported* |
| Offline messages need a persistent session | `bufferTopic()` stores and forwards with the true age of every sample |

### Registers — stock Modbus, for everything else

For the things that are not BeeBus: a SCADA package, a PLC, an HMI, a Modbus
TCP gateway.

```cpp
beebus.addHreg(40, &setpoint);          // Peers may set it
beebus.writeHreg(1, 40, 750);           // Write another node's register
```

Stock function codes, stock addressing. Point a Modbus tool at a gateway and it
sees an ordinary register map. Unlike classic Modbus there is **no master** —
any node reads or writes any other.

**Use topics for your own devices, registers where something else needs to
read you.** The same C++ variable can be both: your nodes see a typed value
with quality, the SCADA sees register 40, and nothing translates between them.

---

## It does the hard part for you

You call `writeHreg()` or `publishAll()` and get on with your application. Between
your call and the wire, and with no configuration at all, the node:

- **Stacks the work.** Operations queue up as you make them. Nothing blocks and
  nothing is sent immediately — it waits for this node's turn with the token.
- **Discards what is redundant.** Queue the same write twice and the second is
  dropped. Ask for the same read twice and so is that.
- **Sends only the latest.** Write a setpoint faster than the token comes round
  and the queued value is *replaced*, not stacked behind. A control loop moving
  a setpoint every scan sends the value it settled on, not a history of every
  step. Whoever was waiting on the replaced write is told, rather than left
  hanging.
- **Packs one frame.** When the token arrives, everything outstanding —
  register writes, reads, topic publishes, replies to other nodes, for *several
  different destinations* — goes into a single frame. One turn on the bus, not
  one per operation.
- **Times itself.** Every timeout is a number of token rotations, and a rotation
  is measured. The same code suits a 115200 wired segment and a long-range radio
  that takes two seconds to come round.
- **Retries and reports.** A confirmed write is retried until it lands or gives
  up, and either way you are told which.

There is nothing to tune. `BeeBusConfig` exists so you *can* pin something on a
segment that demonstrably needs it, and a node started with no config at all is
a fully working node.

---

## The problem

Put four devices on one RS485 pair and you immediately have three problems that
have nothing to do with what you were building.

**Who is allowed to talk?** The usual answer is a master that polls everybody.
Now the master is a single point of failure, every reading is as old as the poll
cycle, and two field devices cannot talk to each other at all.

**How fast is fast enough?** Pick timeouts for a wired segment and they expire
constantly over a radio. Pick them for the radio and a wired fault takes half a
minute to notice. And the right answer changes when you add a node.

**What do these bytes mean?** A Modbus register is sixteen anonymous bits.
Nothing on the wire says whether it is an integer, half a float, or two
characters of a fault message. That lives in a spreadsheet, and the spreadsheet
is always slightly wrong.

## What BeeBus does about it

**Nobody is in charge.** Nodes find each other and form a ring, sorted by node
ID. A token goes round it. Any node can leave or arrive at any time and the ring
closes over the change by itself — including the lowest-numbered one, because
there is nothing special about it.

**The network times itself.** Every timeout is a number of token rotations, and
a rotation is *measured*. Add a node, move to a slower link, and everything
adjusts on its own.

**Values carry their own type.** You bind a `float` and a float is what travels.
The other end checks it against what it bound and refuses a mismatch — so the
spreadsheet stops being the thing that defines the wire.

---

## Quick start: topics

**What it solves:** getting a value from one node to another, continuously,
without polling — and knowing whether you should still believe it.

The node with the value:

```cpp
#include <BeeBusGlobal.h>

BeeBusAPI beebus;
float temperature = 0.0f;

void setup() {
    Serial1.begin(115200);
    beebus.begin(&Serial1, 1, BeeBusConfig(), 115200);      // I am node 1

    beebus.addInputTopic("temperature", 1, &temperature);   // Here is my data

    beebus.connect();
}

void loop() {
    beebus.loop();
    temperature = readSensor();                             // Your own code. That is all
}
```

The node that wants it:

```cpp
#include <BeeBusGlobal.h>

BeeBusAPI beebus;
float remoteTemp = 0.0f;

void setup() {
    Serial1.begin(115200);
    beebus.begin(&Serial1, 2, BeeBusConfig(), 115200);      // I am node 2

    beebus.registerTopic(1, "temperature", 1, &remoteTemp); // Node 1's data, here

    beebus.onMemberJoined([](uint8_t node) {
        if (node == 1) {
            // Tell me when it moves 0.5°, and in any case once a minute
            beebus.subscribe(1, "temperature", 0.5f, 60000);
        }
    });

    beebus.connect();
}

void loop() {
    beebus.loop();

    // remoteTemp now tracks node 1. Nothing polls it.
    if (beebus.topicQuality(1, "temperature") == BeeBusQuality::Good) {
        control(remoteTemp);                                // Safe to act on
    }
}
```

That is the whole thing. No polling loop, no message building, no timeout
constants, no master.

You also get, for free:

- **`topicQuality()`** — `Good`, `Uncertain`, `Bad`, judged against the interval
  *you* asked for. A variable holding 21.5 tells you nothing about whether 21.5
  is still true.
- **Gap reporting** — every update carries a sequence number, so a lost one is
  *reported*, not merely absent.
- **Store-and-forward** — `beebus.bufferTopic("temperature", 60)` and a node that
  goes offline delivers its history later, in order, with the true age of every
  sample attached.
- **Text, not just numbers** — bind a `char` buffer and a topic carries a fault
  message. The length travels with it, so nothing at the far end has to know how
  long it is:

  ```cpp
  char fault[32] = "";
  beebus.addInputTopic("fault", 20, fault, sizeof(fault));
  strcpy(fault, "pump 2 overload");
  ```

---

## Quick start: Modbus registers

**What it solves:** letting something that is not BeeBus see your data — a SCADA
package, a PLC, an HMI, a Modbus TCP gateway.

```cpp
#include <BeeBusGlobal.h>

BeeBusAPI beebus;

bool    pumpRun  = false;
int16_t setpoint = 0;
int16_t level    = 0;

void setup() {
    Serial1.begin(115200);

    BeeBusConfig config;
    config.enableFailSafe = true;
    beebus.begin(&Serial1, 1, config, 115200);

    beebus.addCoil(0,  &pumpRun);      // Peers may switch it
    beebus.addHreg(40, &setpoint);     // Peers may set it
    beebus.addIreg(30, &level);        // Peers may only read it

    // If the node commanding this pump goes away, stop it.
    beebus.setCoilFailsafe(0, BeeBusFailsafe::ResetToValue, false);

    beebus.connect();
}

void loop() {
    beebus.loop();

    level = readLevel();
    digitalWrite(PUMP_PIN, pumpRun);
}
```

From another node:

```cpp
beebus.writeHreg(1, 40, 750, BeeBusDelivery::Confirmed);   // And be told it landed
beebus.readIreg(1, 30, myLevel);                           // Arrives on a later token
```

Stock function codes, stock addressing. A Modbus tool pointed at a gateway sees
an ordinary register map.

**And you can have both.** The same C++ variable can be a topic *and* a holding
register — your nodes read it as a typed value with quality, the SCADA reads
register 40, and nothing translates between them.

---

## Features

| | |
| --- | --- |
| **Peer to peer** | No master, no broker, no coordinator. Any node talks to any node. |
| **Self-organising** | Nodes discover each other and build the ring. Joining and leaving are ordinary events. |
| **Self-timing** | Every timeout is measured, not configured. One config suits wire and radio. |
| **Typed data** | Values carry their type — including text, length-prefixed. Mismatches are refused, not misread. |
| **Publish / subscribe** | Per-subscriber deadband and maximum interval. Two subscribers can ask for different things. |
| **Broadcast** | `publishAll()` reports a value to the whole ring in one section, however many are listening. |
| **Data quality** | `Good` / `Uncertain` / `Bad`, judged against the contract you set. |
| **Gap detection** | Sequence numbers on every publish. A loss is reported, not silent. |
| **Store and forward** | Buffered topics deliver history with true ages after an outage. |
| **Modbus compatible** | Coils, discrete inputs, holding and input registers, plus strings. |
| **Failsafe** | Per-address safe states applied when the commanding node disappears. |
| **Efficient framing** | One frame carries operations for several destinations in one token turn. |
| **Automatic queuing** | Operations stack, duplicates are dropped, and a repeated write sends only the latest value. |
| **Network health** | Every node times every hop, so you can walk the ring and see which link is slow or going bad. |
| **Transport agnostic** | RS485, UDP multicast, radio, or a transport of your own in six methods. |

---

## Supported platforms

| | |
| --- | --- |
| **Arduino** | ESP32, ESP8266, SAMD, STM32, Arduino's mbed-based boards and the Uno R4 (`renesas_uno`) |
| **Host (CMake, C++17)** | Linux, macOS and Windows, with serial and UDP multicast transports |

Arduino boards need **32 KB of RAM or more**. A node needs roughly 13 KB for its
fixed tables and buffers, plus its operation queues, before any of your own
data, so the 8-bit AVR boards (Uno, Nano, Mega) are not supported. See
[Memory, and trimming it](docs/10-configuration.md#memory-and-trimming-it).

---

## Install

**Arduino** — copy this repository into your `libraries` folder, or add it as a
ZIP library. Then:

```cpp
#include <BeeBusGlobal.h>
```

A serial node needs nothing else: `begin(&Serial1, ...)` wraps the port for you.
Other transports have their own header under `transport/`, for example
`#include <transport/BeeBusArduinoUdpTransport.h>`.

**CMake** — add the repository to your project and link the `BeeBus::beebus`
target:

```cmake
add_subdirectory(modbee-beebus)
target_link_libraries(my_app PRIVATE BeeBus::beebus)
```

The library builds as the static target `beebus` (alias `BeeBus::beebus`).
`BEEBUS_BUILD_EXAMPLES` builds the host examples `beebus_serial_node`,
`beebus_udp_node`, `beebus_topic_node` and `beebus_gateway_node`; it defaults on
when BeeBus is the top-level project and off under `add_subdirectory()`.
`BEEBUS_SANITIZE` and `BEEBUS_SANITIZE_THREAD` build with the
address/undefined-behaviour or thread sanitizers.

**Try it with no hardware** — two processes on one machine over UDP multicast is
a real BeeBus network:

```bash
cmake -S . -B build && cmake --build build -j
./build/beebus_topic_node 1 2      # in one terminal
./build/beebus_topic_node 2 1      # in another
```

---

## Documentation

Full documentation is in **[docs/](docs/)**.

| | |
| --- | --- |
| [Overview](docs/01-overview.md) | What it is, what it solves, when not to use it |
| [Getting started](docs/02-getting-started.md) | A first network, one step at a time, with the reason for each step |
| [How the ring works](docs/03-how-the-ring-works.md) | Token passing, membership, healing, measured timing |
| [Frame format](docs/04-frame-format.md) | Every byte |
| [The topic PDU](docs/05-topic-pdu.md) | Typed values, subscriptions, quality, buffering |
| [The Modbus PDU](docs/06-modbus-pdu.md) | Registers, strings, failsafe |
| [Transports](docs/07-transports.md) | RS485, UDP, radio, writing your own |
| [API reference](docs/08-api-reference.md) | Every call |
| [Operations and framing](docs/09-operations.md) | Queuing, batching, acknowledgement, retries |
| [Configuration](docs/10-configuration.md) | Settings and build options |
| [Gateways](docs/11-gateways.md) | Joining segments, and bridging to MQTT |
| [Best practices](docs/12-best-practices.md) | What to do, and what catches people |
| [Applications](docs/13-applications.md) | Six example networks, laid out and explained |

## Examples

| | |
| --- | --- |
| `examples/arduino/Topics_Node1` / `Node2` | Two nodes exchanging typed values |
| `examples/arduino/Topics_Gateway` | RS485 to UDP, on one ESP32 |
| `examples/arduino/Topics_MQTT_Gateway` | BeeBus to an MQTT broker, both directions |
| `examples/arduino/Node1_Remote_IO` / `Node2` | Transparent I/O over registers |
| `examples/arduino/BeeBusProtocol` | A single node with a full register map |
| `examples/arduino/Node_UDP` | A node over WiFi UDP multicast, on an ESP32 |
| `examples/cpp/topic_node` | Topics over UDP, host build |
| `examples/cpp/gateway_node` | RS485 to UDP, host build |
| `examples/cpp/serial_node`, `udp_node` | Registers over serial and UDP |

---

## Not for

Hard real-time motion control. Networks beyond 254 nodes. Untrusted networks —
there is no authentication or encryption, so keep it on a segment you own. Bulk
transfer; a frame is 512 bytes.

**32-bit targets only.** A node needs roughly 13 KB of RAM for its fixed tables
and buffers, plus its operation queues, before your own data, so the 8-bit AVR
boards are out of reach. The largest fixed item is the per-peer hop table, which
covers the whole address space by default and can be
[sized down](docs/10-configuration.md#beebus_hop_table_nodes) to what your ring
actually uses — `-DBEEBUS_HOP_TABLE_NODES=32` buys back 2.6 KB. The queues can
be trimmed with `maxPendingOps` and `maxPendingResponses`.

---

## Licence

BeeBus is licensed under the GNU Affero General Public License v3.0
(AGPL-3.0). See [LICENSE](LICENSE).

---

**You focus on the data. Not the protocol itself.**
</content>
