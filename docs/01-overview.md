# 01 - Overview

## What BeeBus is

BeeBus is a protocol for small networks of devices that all need to talk to each
other, over links that are shared, slow, and not always reliable — RS485, radio,
UDP on a plant LAN.

Every node is equal. There is no master, no server, no broker, and no
coordinator. Nodes take turns to speak by passing a token around a ring they
build themselves, and while a node holds the token it says everything it has to
say, to everybody it needs to say it to, in one frame.

It ships as a C++ library. On Arduino it is a library you install; on Linux,
macOS and Windows it is a CMake target. The same source runs on both.

## The problem it solves

Put four devices on one RS485 pair and you immediately have three problems that
have nothing to do with your application.

**Who is allowed to talk?** Two transmitters on one pair destroy each other's
frames. The usual answer is to appoint a master and let it poll everybody. That
works, and it costs you: the master is a single point of failure, every reading
is as old as the poll cycle, and two field devices cannot talk to each other at
all — everything has to go up to the master and back down.

**How fast is fast enough?** Every protocol has timeouts. Pick them for a wired
segment and they expire constantly over a radio; pick them for the radio and a
wired fault takes half a minute to notice. And the right answer changes when you
add a node.

**What do these bytes mean?** A Modbus register is sixteen anonymous bits.
Nothing on the wire says whether register 40001 is a signed integer, half of a
float, or two characters of a fault message. That knowledge lives in a
spreadsheet, and the spreadsheet is always slightly wrong.

BeeBus's answers:

**Nobody is in charge.** Nodes discover each other and form a logical ring
sorted by node ID. The token goes round it. Any node may leave or arrive at any
time and the ring closes over the change by itself. Lose the node that happened
to be lowest-numbered and the network keeps running — there is nothing special
about it.

**The network times itself.** Every timeout in the protocol is expressed as a
number of token rotations, and a rotation is measured rather than configured.
The same code, unchanged, runs a four-node wired segment that goes round in two
milliseconds and a radio link that takes two seconds. Add a fifth node and every
timeout adjusts on its own. See [03 - How the ring works](03-how-the-ring-works.md).

**Values carry their own type.** You bind a `float` and a float is what travels;
the receiving end checks it against what it bound and refuses a mismatch. See
[05 - The topic PDU](05-topic-pdu.md).

## Two data models, one network

BeeBus carries two kinds of payload, mixed freely in the same frame and on the
same segment.

### Topics — the native model

A topic is a variable on the node that owns it. You bind it, name it, and other
nodes subscribe.

```cpp
// On the node that has the value
float pressure = 0.0f;
beebus.addInputTopic("pressure", 1, &pressure);

// On a node that wants it
float pumpPressure = 0.0f;
beebus.registerTopic(PUMP, "pressure", 1, &pumpPressure);
beebus.subscribe(PUMP, "pressure", 0.5f, 60000);   // ±0.5 bar, or at least once a minute
```

From then on `pumpPressure` tracks the pump. Nothing polls. The pump reports
when the value moves half a bar, and in any case once a minute so silence is
never ambiguous. You can ask whether the value is trustworthy:

```cpp
if (beebus.topicQuality(PUMP, "pressure") == BeeBusQuality::Good) { ... }
```

Topics also give you sequence numbers (so a lost update is *reported*, not
merely absent), true value ages, and store-and-forward for nodes that go offline
and come back.

### Registers — the compatibility model

The same library speaks stock Modbus coils, discrete inputs, holding registers
and input registers.

```cpp
int16_t setpoint = 0;
beebus.addHreg(100, &setpoint);                  // Peers may read and write it
beebus.writeHreg(2, 100, 750, BeeBusDelivery::Confirmed);
```

Use registers when something outside BeeBus has to see the data — a SCADA
package, a PLC, a Modbus TCP gateway, an existing HMI. The address map is a
Modbus address map and behaves like one.

**Both at once is normal.** The same C++ variable can be a topic *and* a holding
register. Your own nodes read it as a typed topic with quality and freshness;
the SCADA reads register 100 and never knows the difference.

### Which to reach for

| | Topics | Registers |
| --- | --- | --- |
| Type on the wire | Yes, checked | No, sixteen bits |
| Addressed by | Name (ID on the wire) | Numeric address |
| Update style | Publish on change, subscriber's terms | Read and write on demand |
| Knows when it is stale | Yes | No |
| Reports a lost update | Yes | No |
| Survives an offline period | Yes, buffered | No |
| Readable by third-party Modbus tools | No | Yes |

Default to topics. Add registers where something outside the network has to
look in.

## When BeeBus is not the right answer

Being straight about this saves everybody time.

- **Hard real-time motion control.** BeeBus is deterministic in the sense that
  every node gets its turn, but the turn arrives when the token does. If you
  need a value on a bounded microsecond schedule, use a fieldbus built for it.
- **Very large networks.** The address space is 1–254 nodes, and every node
  hears every frame. A hundred chatty nodes on one RS485 pair will be
  unsatisfying regardless of protocol.
- **Untrusted networks.** There is no authentication and no encryption. BeeBus
  assumes the segment is yours. On UDP, keep it on a network you control.
- **Bulk transfer.** A frame is 512 bytes. This is a protocol for process values
  and commands, not for moving firmware images.

## What it does not take from MQTT

The topic layer borrows MQTT's vocabulary because it is the vocabulary people
already have. It deliberately does not borrow the rest:

| | |
| --- | --- |
| No broker | Nodes talk directly. There is nothing to install and nothing to fail. |
| No `+` / `#` wildcards | Names are flat, not `/`-separated levels. Wildcards are globs: `*` and `?`. |
| No QoS 2 | Every BeeBus write is idempotent and the ring already orders delivery between a pair of nodes. `Unconfirmed` and `Confirmed` are QoS 0 and 1. |
| No will messages | Membership is the liveness signal, and it is continuous rather than an event. |
| No retained flag | Every topic is a live variable, so it is inherently retained: a new subscriber is sent the current value immediately. |

## What a node costs

**32-bit targets only.** A node needs roughly **13 KB of RAM** for its fixed
tables and buffers, plus its operation queues, before any of your own data, so
the 8-bit AVR boards — Uno, Nano, Mega — are out of reach. The Arduino library
targets ESP32, ESP8266, SAMD, STM32, Arduino's mbed-based boards and the Uno R4
(`renesas_uno`).

Measured on a 32-bit build, the cost of one node:

| | |
| --- | --- |
| Per-peer hop table | **3060 bytes** — 12 per node ID, and [reducible](10-configuration.md) |
| Receive and transmit buffers | 512 bytes each, plus a 5-frame receive queue |
| Liveness table | 1024 bytes, indexed by node ID |
| Operation and response queues | About 300 bytes a slot; 16 of each by default on a board, 50 on a host, and [adjustable](10-configuration.md) |
| Per local topic | 44 bytes on a 32-bit target, 64 on a 64-bit host |
| Per remote topic | 28 bytes / 40 bytes |
| Per subscriber of a topic | 28 bytes |
| Per buffered sample | 16 bytes |
| Registers | One map entry per bound address |

A two-instance gateway carrying both a serial and a UDP segment pays that
twice — comfortable on an ESP32, not a sketch for a board with 2 KB.

## Next

[02 - Getting started](02-getting-started.md) builds a working two-node network
from nothing, and explains why each step is there.
</content>
