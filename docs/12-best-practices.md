# 12 - Best practices

Two halves: what to do, and what actually catches people.

---

# Part one — what to do

## Node IDs

**Assign them deliberately.** The ID decides where a node sits in the ring,
because the token always goes to the next-highest ID.

- **Leave gaps.** `10, 20, 30, 40` rather than `1, 2, 3, 4`. Adding node 25 later
  puts it exactly where you want it in the rotation.
- **Group by segment or by function.** 1–19 field devices, 20–39 controllers,
  40–59 supervision, 90+ gateways and diagnostics.
- **Put a slow node where it does least harm.** BeeBus gives a node with a very
  slow hop the token less often, but a radio node sandwiched between two fast
  wired nodes still costs the ring two slow hops per rotation instead of one.
- **Keep nodes that talk to each other adjacent** where you can. A value
  published by node 10 to node 11 goes out on the very next hop.
- **Write the map down** and keep it with the drawings. A duplicate ID is
  detected and reported, but finding which panel it is in is a site visit.

## Topic IDs and names

- **Keep IDs under 128.** They encode in one byte instead of two. On a radio link
  that is worth doing everywhere.
- **Give the same *kind* of thing the same ID on every node.** If topic 1 is the
  primary measurement on every sensor, a supervisor's code becomes a loop rather
  than a list.
- **Name for the reader, not the wiring.** `pressure`, not `ai_ch3_scaled`.
  Names cost nothing on the wire — they only travel during commissioning.
- **Never renumber a topic in a running plant** without changing both ends
  together. The ID is the identity; the name is a label.

## Choosing a deadband

The deadband is the single biggest lever you have over network load.

**Set it to the smallest change you would actually act on.** Not the sensor's
resolution, and not zero.

| | |
| --- | --- |
| Tank level, 0–5 m | `0.01` (1 cm) — you would not act on a millimetre |
| Line pressure, 0–10 bar | `0.05` |
| Ambient temperature | `0.1` |
| Motor current, 0–100 A | `0.5` |
| Flow rate, noisy | `2.0` — noise below this is not information |
| Counter | `1.0`, or larger if you only want progress |
| Boolean or state | `0.0` — every change matters |

Two rules of thumb:

- **A deadband below the sensor's noise floor is a load generator.** It publishes
  the noise, at whatever rate the noise moves, for ever.
- **Different subscribers should ask for different things.** A local controller
  wants 0.01; a supervisor five kilometres away over a radio wants 0.5. Both get
  exactly what they ask for.

## Choosing a maximum interval

**Always set one on a measurement.** An interval of 0 means the value has no
promise attached, so quality collapses to "is the publisher in the ring", and a
sensor that froze at a plausible reading looks perfectly healthy.

| | |
| --- | --- |
| Fast control input | 1000–5000 ms |
| Ordinary process value | 30000–60000 ms |
| Slow measurement (tank level, weather) | 300000 ms (5 min) |
| Radio telemetry | 600000 ms (10 min) or more |
| Event or command | `0` — legitimate here, because there is nothing periodic to promise |

Remember what the number *means*: quality goes `Uncertain` past the interval and
`Bad` past twice it. Pick it as "how long may this value be silent before I stop
trusting it", because that is exactly what it is.

## Confirmed or unconfirmed

| Confirmed | Unconfirmed |
| --- | --- |
| Setpoints | Periodic telemetry |
| Commands (start, stop, reset) | Anything on a deadband |
| Interlocks and permissives | Status re-sent anyway |
| Anything from an operator or a broker | Anything where the next update is along shortly |
| Anything you would otherwise verify by reading back | |

An ack per reading is airtime the bus does not need. An unacknowledged setpoint
is a fault you find when the plant misbehaves.

**And handle the failure**, or the confirmation bought you nothing:

```cpp
beebus.onWriteComplete([](uint8_t node, uint16_t addr, BeeBusResult result) {
    if (result != BeeBusResult::Immediate) {
        raiseAlarm("command not delivered", node, addr, bb_resultName(result));
    }
});
```

## Always check quality before acting

```cpp
if (beebus.topicQuality(SENSOR, "level") == BeeBusQuality::Good) {
    control(level);
} else {
    holdLastSafeAction();
}
```

**A variable holding 21.5 tells you nothing about whether 21.5 is still true.**
This is the most valuable three lines in the library and the most commonly
skipped.

Decide explicitly what `Uncertain` means for your process — usually "keep going
but flag it", sometimes "same as Bad".

## Bind, then subscribe, then connect

```cpp
void setup() {
    beebus.begin(&Serial1, NODE_ID);

    // 1. Bind everything
    beebus.addInputTopic("pressure", 1, &pressure);
    beebus.registerTopic(PUMP, "flow", 1, &pumpFlow);

    // 2. Register handlers
    beebus.onTopicChanged(...);
    beebus.onError(...);

    // 3. Subscriptions in onMemberJoined, so rejoins are covered
    beebus.onMemberJoined([](uint8_t node) {
        if (node == PUMP) beebus.subscribe(PUMP, "flow", 0.5f, 30000);
    });

    // 4. Last
    beebus.connect();
}
```

The order matters: as soon as the node joins the ring after `connect()`, peers
can write to it and values can start arriving.

## Enable failsafe on anything driving outputs

```cpp
BeeBusConfig config;
config.enableFailSafe = true;
beebus.begin(&transport, NODE_ID, config);

beebus.setCoilFailsafe(0,  BeeBusFailsafe::ResetToValue, false);
beebus.setHregFailsafe(40, BeeBusFailsafe::ResetToValue, 500);

beebus.onFailsafeApplied([](BeeBusRegisterType, uint16_t addr, uint8_t lost) {
    logAlarm("failsafe applied", addr, lost);
});
```

**Register the handler.** Without it, an output driven to safe is
indistinguishable from one deliberately commanded, which is a dangerous
ambiguity in an alarm log.

Think about each address separately. `HoldLastValue` is right for a modulating
valve mid-stroke and wrong for a start command.

**Failsafe acts on the register layer only.** The data map records which node
last wrote each address, and only register-path writes are recorded — a topic
write stores straight into the variable and records no writer. So **command
failsafed outputs over coils and registers**, and use topics for measurement,
status and anything you want typed and quality-stamped. If you prefer to command
over topics, do the equivalent yourself from `topicQuality()`.

## Keep handlers short

Every handler runs inside `loop()`, while the frame that carried the value is
being processed. **Time spent there is time the token is not moving.**

```cpp
// Bad — the whole ring waits for the SD card
beebus.onTopicChanged([](uint8_t n, uint16_t t, const BeeBusScalar& v) {
    writeToSdCard(n, t, v);
});

// Good
volatile bool pending = false;
beebus.onTopicChanged([](uint8_t, uint16_t, const BeeBusScalar&) {
    pending = true;
});

void loop() {
    beebus.loop();
    if (pending) { pending = false; writeToSdCard(...); }
}
```

## Call `loop()` often, and never `delay()`

```cpp
// Bad
void loop() {
    beebus.loop();
    delay(100);           // 100 ms of not answering the token
}

// Good
void loop() {
    beebus.loop();

    static uint32_t last = 0;
    if (millis() - last >= 100) {
        last = millis();
        doPeriodicWork();
    }
}
```

A node that stops calling `loop()` stops answering the token. The ring will
route round it, and eventually conclude it has gone.

The same applies to anything else that blocks: a slow `Serial.print` at 9600
baud, a blocking HTTP request, a `while` loop waiting on a sensor.

## Batch register operations

```cpp
// One PDU, one section, one turn
int16_t values[10];
beebus.readHreg(NODE, 100, values);

// Ten PDUs, ten sections, possibly several turns
for (int i = 0; i < 10; i++) beebus.readHreg(NODE, 100 + i, values[i]);
```

Lay your address map out so the things you read together are next to each other.

## Reserve capacity for large maps

```cpp
beebus.reserveTopics(500, 2000);   // local, remote
```

Each insert keeps the map sorted, so adding thousands one at a time is thousands
of moves. Adding in ascending ID order hits the fast path.

## Watch the right numbers

```cpp
bb_time_t rotation = beebus.getRotationTime();      // How fast is this network?
bb_time_t hop      = beebus.getHopTime(PEER);       // Which link is the slow one?
uint16_t  pending  = beebus.getPendingOpCount();    // Am I generating too much?
BeeBusIOStats io   = beebus.getLinkStatistics();    // Is the physical layer sound?
```

Expose these somewhere you can see them. Rotation time trending upward is the
earliest warning of nearly every network problem there is.

---

# Part two — what actually catches people

## The value never arrives

**Check, in this order:**

1. **Is the topic bound at the receiving end?** `registerTopic()` is not
   optional. A value for a topic with nothing bound to it is silently dropped —
   the binding is what states the expected type.
2. **Is the subscription placed?** Check the return value.
   `bb_accepted(beebus.subscribe(...))`. `NotMapped` means step 1.
3. **Do the types match?** Both ends must bind the same type. A mismatch is
   refused and reported through `onError`, but only if you registered a handler.
4. **Are the topic IDs the same at both ends?** The name is local; the ID is what
   travels.
5. **Is the peer actually a member?** `beebus.isMember(PEER)`.

The commonest single cause is placing the subscription in `setup()` before the
peer has joined. Put it in `onMemberJoined`.

## It works, then stops after a restart

Subscriptions live on the **publisher**, and a node that restarts has lost its
subscriber list.

BeeBus re-issues them for you when a peer rejoins. If you have overridden that —
or if you are wondering why it works — this is the mechanism. Placing
subscriptions in `onMemberJoined` covers it either way.

## `writeTopic` completes with `TypeMismatch`

The value's type differs from the one the owner bound, so the owner refuses it
and the write completes `TypeMismatch` (through `onWriteComplete`, or
`onTopicComplete` when no write handler is set). The type comes from the argument: `4.5f` is a `float`;
`(int16_t)4` is an `int16_t`; a bare `4` is an `int`, which is not a fixed-width
type — it fails to compile on boards where `int32_t` is a different type, and
travels as `int32_t` where it is the same.

```cpp
beebus.writeTopic(PUMP, "setpoint", 4.5f);          // float
beebus.writeTopic(PUMP, "enable", true);            // bool
beebus.writeTopic(PUMP, "mode", (int16_t)2);        // int16_t — cast it
```

## `publishAll(0)` does not compile, or does the wrong thing

Topic ID 0 is reserved and always fails: IDs run from `BEEBUS_TOPIC_ID_MIN` (1)
to `BEEBUS_TOPIC_ID_MAX` (32767). `0` is also the null pointer constant, so a
literal `0` is ambiguous between the ID form and the name form of every
overloaded call. Start your IDs at 1.

## Everything is `Unknown` or `Bad`

- `Unknown` — nothing has ever arrived. Work through the list above.
- `Bad` right after the publisher restarts — normal. It left the ring, and its
  first value since rejoining has not landed yet.
- `Bad` intermittently — the publisher is missing its interval. Either loosen
  the interval, or find out why: check `getRotationTime()`, `crcErrors`, and
  whether the publisher is blocking in `loop()`.

## `QueueFull`

You are generating operations faster than the token comes round.

- Loosen deadbands and lengthen intervals.
- Batch register operations.
- Check `getRotationTime()` — if it has grown, something else is wrong.
- Check you are not calling `publishAll()` on a timer for a topic that is already
  subscribed. It is published on the subscriber's terms without you.

## Two nodes with the same ID

Detected and reported as `DuplicateNodeID`. The node already carrying the ring's
traffic keeps the address; the other stands down and takes no part for as long
as the duplicate keeps transmitting. While it is stood down,
`hasDuplicateNodeID()` returns true and every read and write it issues is
refused with `BeeBusResult::DuplicateNodeID`.

It comes back by itself once its address has been quiet for the membership
timeout — usually because the other device was taken off the network — or on
the next `connect()`. The real fix is still to give one of them a new ID.

## The ring keeps re-forming

Symptoms: repeated `MemberJoined` / `MemberLeft`, rotation time jumping about.

- **Termination.** 120 Ω at each end of the RS485 run, and only at the ends. This
  is the most common cause by a wide margin.
- **A node blocking in `loop()`.** It misses token passes, gets routed round, and
  eventually removed. Look for `delay()`, blocking I/O, long handlers.
- **Marginal link.** Check `crcErrors`. Try a lower baud rate.
- **Baud mismatch.** `noiseBytesDiscarded` climbing while `framesReceived` does
  not is the signature.
- **Multicast on UDP.** IGMP snooping on a managed switch can partition the ring
  in ways that look exactly like a flaky cable.

## Values stop crossing a gateway

- Each side is a separate ring with its own membership. Check both.
- The subscription on the far side must be placed against the **gateway's** node
  ID, not the original publisher's.
- Is the gateway calling `loop()` on both instances?
- Quality does not cross. If it looks `Good` but is not, you have not relayed it
  — see [11 - Gateways](11-gateways.md).

## A store-and-forward buffer never flushes

Four gates, all required:

1. The node is taking part in a ring.
2. At least 4 rotations since joining (`rotationsFlushSettle` in
   `BeeBusConfig`; 4 is the default).
3. The operation queue is under half full.
4. The topic has at least one subscriber **that is currently a ring member**.

Gate 4 is the usual culprit: nothing is subscribed on the receiving side, so
there is nowhere to send.

## `onReadComplete` never fires

It applies to reads issued **after** it is set. Register it before you issue
reads.

## Frames arriving but nothing happening

Check `getLinkStatistics()`:

| | |
| --- | --- |
| `framesReceived` rising, nothing else | Frames are for other nodes. Normal. |
| `crcErrors` rising | Corruption — or two nodes answering one solicitation, which is normal during joining. |
| `noiseBytesDiscarded` rising steadily | Floating bus, missing termination, or baud mismatch. |
| `bufferOverflows` rising | Not calling `loop()` often enough. |
| `framingErrors` rising | Runt frames. Usually a physical layer problem. |

## Debugging tools

```cpp
beebus.onError([](BeeBusError error, const char* message) {
    Serial.print(bb_errorName(error)); Serial.print(": "); Serial.println(message);
});
```

**Register this first, on every node, always.** It costs nothing and it reports
membership changes, CRC errors, refused operations and protocol violations.

For a segment that will not behave:

```cpp
beebus.onFrameDebug([](const char* dir, const uint8_t*, uint16_t len, const char* decoded) {
    Serial.print(dir); Serial.print(' '); Serial.print(len); Serial.print(' ');
    Serial.println(decoded);
});
```

This costs a decode per frame. Use it to diagnose, not in production.

Two processes on one machine over UDP multicast reproduce most protocol
behaviour with no hardware at all, and are far easier to instrument than a live
segment:

```bash
./build/beebus_topic_node 1 2
./build/beebus_topic_node 2 1
```

## A short checklist before commissioning

- [ ] Node IDs unique, written down, with gaps left
- [ ] `onError` registered on every node
- [ ] Subscriptions placed in `onMemberJoined`
- [ ] Every measurement has a maximum interval
- [ ] Quality checked before acting on any remote value
- [ ] Commands and setpoints sent `Confirmed`, with `onWriteComplete` handled
- [ ] Failsafe enabled and stated per address on anything driving outputs
- [ ] No `delay()` and no blocking calls in `loop()`
- [ ] Handlers short
- [ ] RS485 terminated at both ends, and only both ends
- [ ] Rotation time and link statistics visible somewhere
</content>
