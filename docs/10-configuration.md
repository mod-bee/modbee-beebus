# 10 - Configuration

The short version: **you do not have to configure anything.** `BeeBusConfig{}`
is the intended way to start a node, and every timing field in it defaults to
"measure this yourself".

This chapter is for the cases where you genuinely need to override something.

## BeeBusConfig

```cpp
struct BeeBusConfig {
    // Bus timing
    bb_time_t interframeGapUs     = BEEBUS_AUTO;
    bb_time_t initialListenPeriod = 2000;

    // Join and token ring
    bb_time_t joinCycleInterval    = BEEBUS_AUTO;
    bb_time_t joinResponseTimeout  = BEEBUS_AUTO;
    bb_time_t tokenResponseTimeout = BEEBUS_AUTO;
    bb_time_t tokenReclaimTimeout  = BEEBUS_AUTO;
    bb_time_t nodeTimeout          = BEEBUS_AUTO;

    // Operations
    bb_time_t operationTimeout = BEEBUS_AUTO;
    bb_time_t responseTimeout  = BEEBUS_AUTO;
    bb_time_t retryDelay       = BEEBUS_AUTO;
    uint32_t  maxRetries       = 2;

    // Ring timing, in rotations
    uint8_t rotationsMembership  = BEEBUS_AUTO;   // 16
    uint8_t rotationsIsolation   = BEEBUS_AUTO;   // 32
    uint8_t rotationsTokenGiveUp = BEEBUS_AUTO;   // 2
    uint8_t rotationsStillThere  = BEEBUS_AUTO;   // 4
    uint8_t rotationsReclaim     = BEEBUS_AUTO;   // 4
    uint8_t rotationsRejoin      = BEEBUS_AUTO;   // 30
    uint8_t rotationsOperation   = BEEBUS_AUTO;   // 16
    uint8_t rotationsRetry       = BEEBUS_AUTO;   // 1
    uint8_t rotationsFlushSettle = BEEBUS_AUTO;   // 4
    uint8_t passesWithoutTurn    = BEEBUS_AUTO;   // 8, per member
    uint8_t rotationsStreamResend = BEEBUS_AUTO;  // 4
    uint8_t rotationsResult      = BEEBUS_AUTO;   // operation timeout x (maxRetries + 2)

    // Reliable topics
    uint8_t  streamWindow = BEEBUS_AUTO;          // 64, range 1..127
    uint16_t sessionId    = BEEBUS_AUTO;          // drawn at random in begin()

    // Discovery
    uint8_t solicitBackupRotations = BEEBUS_AUTO; // 6
    uint8_t solicitSpreadMax       = BEEBUS_AUTO; // 16
    uint8_t announceAttempts       = BEEBUS_AUTO; // 3
    uint8_t announceRearmFrames    = BEEBUS_AUTO; // 8
    uint8_t announceSpreadFrames   = BEEBUS_AUTO; // 4

    // Estimator behaviour
    bb_time_t timingFloorMs      = BEEBUS_AUTO;   // 50
    bb_time_t timingFloorMinMs   = BEEBUS_AUTO;   // 5
    uint8_t   estimatorWarmup    = BEEBUS_AUTO;   // 4
    uint8_t   retrySpreadPercent = BEEBUS_AUTO;   // 50

    // Token share
    uint8_t tokenShareMinRatio = BEEBUS_AUTO;     // 4
    uint8_t tokenShareMax      = BEEBUS_AUTO;     // 4

    // Failsafe
    bool enableFailSafe = false;

    // Queue depths, 1..255
    uint8_t maxPendingOps       = BEEBUS_AUTO;    // 50 on a host, 16 on a board
    uint8_t maxPendingResponses = BEEBUS_AUTO;    // 50 on a host, 16 on a board
};
```

**Nothing here has to be set.** `BEEBUS_AUTO` everywhere is the shipped
behaviour, and a node started with `beebus.begin(&port, 1)` and no config at all
is a fully working node. The comment beside each field is the figure the
protocol was built with, which is what `BEEBUS_AUTO` uses.

`initialListenPeriod`, `maxRetries` and `enableFailSafe` are the fields with a
plain default rather than `BEEBUS_AUTO`. `initialListenPeriod` is one because a
node on a silent bus has nothing to measure: it has to pick some duration before
concluding it is alone.

Durations are **milliseconds** unless the name says otherwise. Settings belong to
the *instance*, not the process, so several nodes in one program can each carry
their own — which is what makes a two-segment gateway work.

```cpp
BeeBusConfig config;
config.enableFailSafe = true;
beebus.begin(&transport, NODE_ID, config);
```

## BEEBUS_AUTO and pinning

`BEEBUS_AUTO` is 0, and it means: **derive this from measurement.**

Setting a field to a number pins it and switches adaptation off **for that field
alone**. The rest of the node still tunes itself.

A pinned field states one member of its family, and the others keep their
proportion to it. Pinning `nodeTimeout` to 30 seconds does not collapse "is this
peer gone" and "has this peer refused the token twice" onto the same test — the
second stays at its designed quarter of the first.

**You should almost never pin anything.** The measured values are better than a
guess, they follow the network as it changes, and they do not need revisiting
when you add a node next year. If you find yourself reaching for these, the
honest first question is whether something else is wrong.

## The settings you might actually change

### `enableFailSafe`

```cpp
config.enableFailSafe = true;
```

Off by default. **Turn it on for anything driving real outputs.**

When a node leaves the ring, every register it last wrote gets the action you
stated for that address. Without stating anything, an address holds its last
value — because driving an output unasked is its own hazard.

See [06 - The Modbus PDU](06-modbus-pdu.md).

### `interframeGapUs`

The minimum silence a node leaves before transmitting.

`BEEBUS_AUTO` takes it from the line: two byte times, and never shorter than the
interval this node looks at the bus in. That is what makes room for a waiting
node's announcement, so the gap is not merely dead air.

```cpp
config.interframeGapUs = 20000;   // 20 ms floor
```

A pinned value is a **floor** rather than a replacement, and is still raised if
the line turns out slower than it assumed. Pin one only where the hardware needs
longer than the line alone accounts for:

- Slow RS485 transceivers that need turnaround time
- Half-duplex radios with a transmit/receive switching delay

Lowering it is not useful: the measured figure is already the shortest gap this
node can reliably observe.

### `initialListenPeriod`

```cpp
config.initialListenPeriod = 5000;
```

How long a node listens for an existing ring before deciding the bus is silent
and starting one of its own. Default **2000 ms**.

The actual wait is **staggered per node ID** — a hash of the ID spreads every
address across the window — so a panel of nodes powered up together do not all
decide the bus is silent at the same instant and all start rings.

Raise it if nodes take a long time to become ready, or if a segment can be
genuinely quiet for several seconds while still having members.

### `maxRetries`

```cpp
config.maxRetries = 4;
```

Default **2**. Applies to operations and to token passes.

Raising it makes the network more patient on a lossy link, at the cost of taking
longer to notice a genuinely dead node — the token reclaim window is derived
from the retry budget, so it grows with this number. Anything above
`BEEBUS_MAX_RETRIES` (255) is read as 255, because the counters that check it
are single bytes.

### `maxPendingOps` and `maxPendingResponses`

```cpp
config.maxPendingOps       = 32;
config.maxPendingResponses = 32;
```

How many operations and how many responses this node can have queued, 1..255.
`BEEBUS_AUTO` is **50** each on a host and **16** each on a board. The queues are
allocated once, at `begin()`, at about 300 bytes a slot, and `begin()` fails
when the memory is not there.

Raise the operation depth on a node that keeps many requests in flight at once:
past it, a request is refused with `QueueFull`. Raise the response depth on a
node that answers many requests at once: an answer past it is not queued. Lower
both on a board that is short of RAM — see [Memory, and trimming it](#memory-and-trimming-it).
See also [09 - Operations and framing](09-operations.md#the-operation-queue).

### `joinCycleInterval` and `joinResponseTimeout`

`BEEBUS_AUTO`, using **50 ms** each. These start the phase that *creates* the
thing every other timeout is derived from, because there is no ring to measure
before one has been built.

Neither stays the operative figure for long. `joinResponseTimeout` is where the
contention window *starts*: the window then widens whenever a gap answers late,
and narrows again on a round nothing collided in, so it finds the room a medium
needs rather than being told. That matters most on a store-and-forward radio,
where the byte time a node can observe belongs to the module's UART rather than
to the air and so cannot correct the figure on its own.

`joinCycleInterval` paces a node that is still alone, and stands in for a
rotation before one has been measured.

There is normally no reason to pin either. Check `getHopTime()` and
`getRotationTime()` first — measurement takes over on its own.

### Rotation counts, discovery and estimator fields

Every one of these is `BEEBUS_AUTO` and takes the built-in figure listed above.
They exist so a segment that demonstrably needs something different can have it
without a rebuild, not because a working network needs them set.

The rotation counts are **dimensionless on purpose**: a rotation is measured, so
none of them changes when a node is added or a link slows down. They are also a
*set* — moving one alone can leave a node declared dead sooner than the ring can
hand it the token. If you pin one, check the others still make sense beside it.

`solicitSpreadMax` bounds how far the response window may widen, and
`announceRearmFrames` how much ring traffic a waiting node hears before it may
announce itself again. `tokenShareMinRatio` and `tokenShareMax` decide when a
slow link is visited less often than the rest.

`streamWindow` is how many RELIABLE values may be in flight to one peer before
further changes wait and coalesce, and `rotationsStreamResend` how long a
RELIABLE stream may go without acknowledgement progress before it is re-sent.
`sessionId` tells a restarted node apart from the instance before it; leave it
at `BEEBUS_AUTO` so each `begin()` draws a fresh one from the platform's random
source.

### The AUTO timing family

`tokenResponseTimeout`, `tokenReclaimTimeout`, `nodeTimeout`,
`operationTimeout`, `responseTimeout`, `retryDelay`.

All measured by default. Pin one only when you have a specific, demonstrated
reason, and read [03 - How the ring works](03-how-the-ring-works.md) first so
you know what you are switching off.

The most defensible reason to pin `nodeTimeout` is a regulatory or process
requirement that a lost node be declared within a stated time. Even then,
measure the rotation time first and pick a number comfortably above 16 rotations.

## Build-time options

### `BEEBUS_TOPIC_NAMES`

```cpp
// In your build flags, so the library is compiled with it too.
-DBEEBUS_TOPIC_NAMES=0
```

Default **1**. Setting it to 0 compiles out:

- The name pointer on every local and remote topic
- Name lookups
- The commissioning list exchange
- Wildcards (`subscribeAll`)

Every named call has an ID-only form beside it, so the API stays complete. Worth
doing on a genuinely tiny target; not worth doing anywhere else, because names
are the thing that makes the code readable.

### `DISABLE_BEEBUS_DEBUG`

```cpp
// In your build flags, so the library is compiled with it too.
-DDISABLE_BEEBUS_DEBUG
```

Compiles out the entire debug system — every message, every formatting buffer,
and frame decoding. `onDebug`/`onFrameDebug`/`onModbusDebug` stay declared and
do nothing, so code that installs a handler builds either way.

**Boards (Arduino) have it out by default**: it costs about 10 KB of flash, and
a board in the field has nobody reading its console. Define
`BEEBUS_ENABLE_DEBUG` to put it back for a bench session. A host build keeps it
unless `DISABLE_BEEBUS_DEBUG` is defined.

### CMake options

| Option | Default | |
| --- | --- | --- |
| `BEEBUS_BUILD_EXAMPLES` | `ON` at top level, `OFF` under `add_subdirectory()` | Build the plain C++ examples |
| `BEEBUS_BUILD_TESTS` | `ON` at top level, `OFF` under `add_subdirectory()` | Build the tests, when a `test/` directory is present |
| `BEEBUS_SANITIZE` | `OFF` | Address and undefined-behaviour sanitizers |
| `BEEBUS_SANITIZE_THREAD` | `OFF` | Thread sanitizer, GCC and Clang only (cannot be combined with `BEEBUS_SANITIZE`) |

```bash
cmake -S . -B build-asan -DBEEBUS_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build-asan -j
```

The library parses bytes it did not produce, so out-of-bounds reads are the
failure mode that matters most and the one least likely to show up as a wrong
answer. Run under the sanitizers if you are modifying the library.

## The clock

The protocol reads time only through `bb_millis()` and `bb_micros()`, and both
dispatch through a replaceable source.

```cpp
bb_time_t myMillis() { return myRtosTickToMs(); }
bb_time_t myMicros() { return myHardwareTimer(); }

bb_setClockSource(myMillis, myMicros);
bb_resetClockSource();                   // Back to the platform clocks
```

Two reasons this exists:

**Porting.** Drive the protocol from an RTOS tick, a hardware timer, or whatever
a bare-metal target provides, without touching the core.

**Simulation.** Advance time deterministically when exercising your own
application, so timing behaviour does not depend on how fast the host happens to
run — and so the 49.7-day rollover can be exercised in milliseconds rather than
in seven weeks.

A source must be **monotonic within its width** and must **wrap rather than
saturate**, matching the platform clocks it replaces.

`bb_time_t` is fixed at 32 bits on every target. `unsigned long` is 32-bit on
AVR, ESP32 and MSVC but 64-bit on Linux and macOS, which would change rollover
behaviour between platforms; pinning the width keeps timeout arithmetic
identical everywhere.

## Compile-time constants

These live in headers rather than in `BeeBusConfig` because changing one changes
the protocol's behaviour rather than tuning it. **Every node on a segment must
agree.**

| Constant | Value | Header |
| --- | --- | --- |
| `BEEBUS_MAX_RX_BUFFER` / `TX_BUFFER` | 512 | `BeeBusTypes.h` |
| `BEEBUS_MAX_RX_BYTES_PER_LOOP` | 1024 | `BeeBusTypes.h` |
| `BEEBUS_TOPIC_SCAN_PER_LOOP` | 32 | `BeeBusTopicMap.h` |
| `BEEBUS_TOPIC_FLUSH_PER_TURN` | 4 | `BeeBusTopicTypes.h` |
| `BEEBUS_TOPIC_BUFFER_MAX` | 127 | `BeeBusTopicMap.h` |
| `BEEBUS_TOPIC_LIST_PAGE_BYTES` | 200 | `BeeBusTopicTypes.h` |
| `BEEBUS_TOPIC_GROWTH_STEP` | 256 | `BeeBusTopicMap.h` |
| `BEEBUS_TURN_HISTORY` | 8 | `BeeBusTypes.h` |
| `BEEBUS_HOP_TABLE_NODES` | 254 | `BeeBusTiming.h` |
| `BEEBUS_MAX_KNOWN_NODES` | 254 | `BeeBusTypes.h` |

Wire format — `BEEBUS_FLAG`, `BEEBUS_HDR_*`, `BEEBUS_TOP_*`, the section masks
and `BEEBUS_NODE_BROADCAST` — is also compile-time, and changing any of it
changes what a frame *is*.

Everything that alters how the protocol *behaves* rather than what a frame is or
what a node can hold has a `BeeBusConfig` field, listed at the top of this
page. The `BEEBUS_*` figure behind each remains in the headers (mostly
`BeeBusTiming.h`, with the stream figures in `BeeBusStreams.h` and the discovery
ones in `BeeBusTypes.h`) as the value `BEEBUS_AUTO` uses.

## Memory, and trimming it

**BeeBus runs on 32-bit targets only.** A node costs roughly **13 KB of RAM** in
fixed structures, plus its operation queues, before any of your own data. That
puts the 8-bit AVR boards — Uno, Nano, Mega — out of reach. The supported
architectures are ESP32, ESP8266, SAMD, mbed-based boards, the Renesas Uno R4
and STM32; on the boards with 32 KB of RAM, trim as described below.

Measured on a 32-bit build, with `BEEBUS_HOP_TABLE_NODES` at its default and the
debug system compiled out:

| | bytes |
| --- | --- |
| `BeeBusProtocol` | 6600 |
| — of which the hop table | 3060 |
| — of which `BeeBusModbusDataMap` | 140 |
| — of which `BeeBusTopicMap` | 68 |
| — of which `BeeBusConfig` | 84 |
| `BeeBusIO`, allocated at `begin()` | 6336 |
| Operation and response queues, allocated at `begin()` at the board default of 16 + 16 slots | about 11600 |

Everything above is taken by the time `begin()` returns, and the queues scale
with `maxPendingOps` and `maxPendingResponses`. Registers, topics, subscribers
and buffered samples are allocated as you bind them, so they grow with what the
application actually uses rather than with the limits.

### `BEEBUS_HOP_TABLE_NODES`

The largest single item, and the one worth trimming first. Every node times
every hop it hears and keeps the estimate against the peer it belongs to, at
**12 bytes per node ID**. The default covers the whole address space, because a
ring may hold any address.

```cpp
// In your build flags, not in BeeBusConfig: it sizes an array.
-DBEEBUS_HOP_TABLE_NODES=32
```

| setting | hop table | `BeeBusProtocol` |
| --- | --- | --- |
| 254 (default) | 3060 | 6600 |
| 32 | 396 | 3936 |
| 16 | 204 | 3744 |

**Assign node IDs from 1 upward if you lower it.** The table is indexed by node
ID, not by position, so a ring of eight nodes numbered 4, 17, 60 and 201 needs a
table of 201 to hold them all.

Lowering it is safe. An address above the table still joins, is still timed, and
is still a full member — it simply shares the pooled estimate instead of holding
one of its own, so its link is timed less precisely. Nothing fails; you lose
resolution on exactly the links that fall outside.

**Leave headroom.** Set it above the number of nodes you expect, not equal to
it, so nodes added later still get their own entry. Going from 254 to 32 buys
back 2.6 KB and still covers four times a typical panel.

### Other fixed costs, if you need more

| | default | effect of lowering |
| --- | --- | --- |
| `BEEBUS_MAX_RX_BUFFER` / `TX_BUFFER` | 512 | The receive, transmit and processing buffers and the five-frame receive queue scale with these, about 5 KB of `BeeBusIO`. Caps the largest frame a node can send or receive, so **every node on the segment must agree**. |
| `BEEBUS_MAX_PENDING_OPS` | 50 | Only the default queue depth on a host; a board's is 16. Set the depth per node with `maxPendingOps` / `maxPendingResponses` instead (1..255). At about 300 bytes a slot plus bookkeeping, dropping a board from 16 + 16 to 4 + 4 saves roughly 8.7 KB; a busy gateway wants the headroom instead. |
| `BEEBUS_TURN_HISTORY` | 8 | 24 bytes. Not worth trimming, and shortening it weakens duplicate-turn detection. |

The two node-ID-indexed tables that are *not* reducible are liveness
(`_lastHeardFrom`, 1024 bytes) and the last turn seen per peer (256 bytes).
Both must cover the whole address space to stay correct for any node ID, and
neither degrades gracefully the way the hop table does.

## Configuration by scenario

### A wired panel, a handful of nodes

```cpp
BeeBusConfig config;
config.enableFailSafe = true;
```

That is genuinely it.

### A long RS485 run in a noisy site

```cpp
BeeBusConfig config;
config.enableFailSafe = true;
config.maxRetries     = 4;      // More patient on a lossy link
```

Drop the baud rate to 19200 as well. That buys more than any timing change will.

### A radio segment

```cpp
BeeBusConfig config;
config.enableFailSafe      = true;
config.initialListenPeriod = 10000; // Joining takes longer out here
config.maxRetries          = 4;
```

The timing needs nothing. The gap comes from the line, and the response window
widens by however much answering out here turns out to cost — which on a radio
is the whole point. Spend your effort on deadbands and intervals instead; see
[12 - Best practices](12-best-practices.md).

### A UDP segment on a plant LAN

```cpp
BeeBusConfig config;
config.enableFailSafe = true;
```

There is no half-duplex turnaround to allow for, and the node works that out
from the line rather than being told.

### Several nodes in one process

Settings belong to the instance, so a gateway can hold one node on each side
with different policies. Neither needs its timing stated.

```cpp
BeeBusConfig serialConfig;
serialConfig.enableFailSafe = true;   // This side drives real outputs

BeeBusConfig udpConfig;               // This side only reports

serialSide.begin(&serialTransport, NODE_ID, serialConfig);
udpSide.begin(&udpTransport, NODE_ID, udpConfig);
```

Each instance carries its own config and measures its own timing. Neither shares
a timeout with the other.
</content>
