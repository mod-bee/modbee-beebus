# 09 - Operations and framing

What happens between calling `writeTopic()` and the bytes leaving the wire — and
why the answer is what makes BeeBus efficient on slow links.

## The whole path

```
   your call                    while holding the token             at the far end
   ─────────                    ───────────────────────             ──────────────
   writeTopic()  ──┐
   readHreg()    ──┼──►  the operation queue  ──►  frame builder ──►  one frame  ──► parsed
   publishTo()   ──┤      (50 host, 16 board)      packs sections                    per section
   writeString() ──┘                                                                 │
                                                                                     ▼
   onWriteComplete  ◄──  acknowledgement  ◄──  the reply queue  ◄────────────  handler
   onReadComplete        or response            (50 host, 16 board)
```

Nothing you call transmits. Everything queues, and the queue is emptied when
this node holds the token.

**Why:** on a shared bus only one node may transmit at a time. Queuing is not a
delay imposed by the library — it is the delay the medium imposes, made
explicit. And because everything queues, several operations naturally end up in
one frame, which is where the efficiency comes from.

## Frame packing

This is the part worth understanding.

When a node takes the token, it walks its queues and packs as many operations as
will fit into a single frame, each in its own length-prefixed section, each
independently addressed.

```
┌────────┬───────────┬───────────┬───────────┬───────────┬─────┐
│ HEADER │ sec → n4  │ sec → n7  │ sec → n7  │ sec → n20 │ CRC │
└────────┴───────────┴───────────┴───────────┴───────────┴─────┘
  token     publish     setpoint    register     read
  to n4                             write        reply
```

**One frame. One turn. Four destinations.**

Compare with polling. A master reading four slaves issues four requests and
waits for four responses: eight frames, eight turnarounds, and every reading is
as old as the poll cycle. BeeBus does the same work in one frame each way, and
the readings arrive when they change rather than when it is their turn to be
asked.

On RS485 the turnaround dominates everything at low baud rates, so this is not a
marginal optimisation. On a radio with a transmit/receive switching delay it is
the difference between usable and not.

### How the builder decides

The frame is built in this order:

1. **Header**, including the token pass to the successor.
2. **Pending operations** — writes, reads, publishes, subscriptions, in queue
   order.
3. **Pending responses** — read replies, acknowledgements, `SUBACK`s, `ERROR`s,
   list pages.

For each candidate, four things are checked:

| | |
| --- | --- |
| Already in flight? | A read awaiting its answer must not be sent again. Skip. |
| Encodable at all? | An operation whose function code cannot be built never will be. It is **dropped** with an error, not left to block everything behind it. |
| Retry backoff elapsed? | A retry that goes again on the very next token repeats the attempt into whatever swallowed the first one. Skip until its backoff is done. |
| Does it fit? | Two bytes of section header, one destination byte, the PDU, and two bytes kept back for the CRC. |

When a section will not fit, **the build stops** — it does not skip ahead to a
smaller one. Reordering would break the queue's ordering guarantees for no real
gain.

### Retiring what went out

Only what actually went into the frame is retired, and each kind differently:

| | |
| --- | --- |
| Unconfirmed write | **Finished** the moment it is transmitted. |
| Confirmed write | Kept, awaiting acknowledgement, so it can be retried. |
| Read | Kept, so its response can be matched back to your variable. |
| Reply / acknowledgement | Removed once sent. |

An operation left behind because it did not fit **stays queued**. You were told
it was accepted, so dropping it would lose a write.

### Sizing

| | |
| --- | --- |
| Frame | 512 bytes |
| Section overhead | 3 bytes (2 header + 1 destination) |
| Confirmed write adds | 2 bytes for the operation ID |
| Topic publish (`f32`, ID < 128) | 8 bytes of PDU |
| Modbus single register write | 5 bytes of PDU |
| Acknowledgement entry | 3 bytes, up to 32 in one section |

A `publishAll()` costs one section however many nodes are subscribed, where
`publishTo()` costs one per named peer. On a node reporting a handful of values
to several listeners that is the difference between a frame and several.

A frame comfortably carries dozens of topic publishes. In practice you run out
of things to say long before you run out of frame.

## The operation queue

By default **50 pending operations** and **50 pending responses** on a host,
**16** each on a board. `BeeBusConfig::maxPendingOps` and `maxPendingResponses`
set any depth up to 255, taken once at `begin()` (about 300 bytes a slot);
`begin()` fails and says so when the memory is not there. Past the depth, new
operations are refused with `QueueFull`. A node that answers many requests at
once (a peer subscribing to many topics in one go) wants its response depth as
deep as the asker's operation depth: an answer past it is not queued.

```cpp
uint16_t depth = beebus.getPendingOpCount();
```

A queue that sits near its depth means you are generating work faster than the token
comes round. The fix is nearly always to publish less often — a looser deadband,
a longer interval, block operations instead of singles — rather than a bigger
queue.

Some parts of the library deliberately stay under **half** the queue, so
application traffic always gets through:

- Store-and-forward flushes
- Deferred subscriptions (rejoins, wildcard expansions)

## Superseding

Queue a write to an address that already has one pending, and the new value
**replaces** the old one in place.

```cpp
beebus.writeHreg(PUMP, 40, 500);    // Queued
beebus.writeHreg(PUMP, 40, 520);    // Replaces it. One frame, one value: 520.
beebus.writeHreg(PUMP, 40, 540);    // Replaces it again. 540 goes out.
```

**Why this is right:** you are saying what the register *should be*, not queuing
a history of what it should have been. A control loop updating a setpoint faster
than the token rotates does this constantly. Rejecting the newer write would
leave the register holding a value the application has already moved on from,
and queuing all three would spend three sections saying the same thing.

The replacement takes **every** field the new operation carries, not just its
payload — delivery mode and completion handler included. A confirmed write
replacing an unconfirmed one must not inherit the old one's terms.

Anything waiting on the replaced operation is told:

```cpp
beebus.onWriteComplete([](uint8_t node, uint16_t address, BeeBusResult result) {
    if (result == BeeBusResult::Superseded) {
        // Never reached the wire. A newer value did.
    }
});
```

### When superseding does *not* happen

Four cases, each for a reason:

- **Identical value and same delivery mode** → `Duplicate`. Nothing to do.
- **Different PDU families** never supersede each other. `0x06` is a Modbus
  single-register write *and* a topic `UNSUBSCRIBE`; they are different
  operations whatever the numbers happen to be, and the family is checked
  before anything else, so a topic operation can never replace a register write
  or the other way round.
- **Already awaiting acknowledgement.** The write is on the wire. Rewriting its
  payload now would leave the acknowledgement, when it arrives, confirming a
  value that was never transmitted. The new write queues separately behind it.
  A topic write that has carried its request ID onto the wire counts as on the
  wire while it waits to be retried, too: it may have been applied.
- **Publishes and buffered samples never supersede.** Each carries its own
  sequence number, and collapsing two would leave a number on the wire that
  nothing was ever sent under — which the receiver would read as a gap.

Reads never supersede either: the same read twice is genuinely redundant, and
the second returns `Duplicate`.

## Confirmed delivery

```cpp
beebus.writeHreg(PUMP, 40, 500, BeeBusDelivery::Confirmed);
beebus.writeTopic(PUMP, "setpoint", 4.5f, BeeBusDelivery::Confirmed);
```

A confirmed operation carries a 16-bit **operation ID** after its PDU, with the
section's `ACK` bit set. The receiver echoes that ID back with a status.

**Why an ID and not the address:** destination, function and address do not
identify an operation. A write can be in flight while a superseding write to the
same register waits behind it, and matching on the address would retire the
wrong one.

The acknowledgement travels **whatever the outcome**. A refusal carries the
Modbus exception code that caused it, because the sender is waiting on that ID
and will keep retrying until it hears something.

Up to 32 acknowledgements to one node share a section, so several confirmed
writes in one token turn cost 3 bytes each rather than a section apiece.

**Applied once.** A confirmed topic write also carries a **request ID** in its
PDU (per writer → owner pair, given as it first goes into a frame and kept for
its retries) and the writer's session. The owner remembers the last 32 request
IDs from each writer and applies each once: a retry after a lost
acknowledgement is acknowledged again and not applied again. A writer that restarts has a new session, which
clears its window at the owner. Register writes keep the plain operation ID.

### Which to use

| Use `Confirmed` for | Use `Unconfirmed` for |
| --- | --- |
| Setpoints and commands | Periodic telemetry |
| Interlocks and safety-related writes | Anything published on a deadband |
| Anything you would otherwise verify by reading back | Anything where the next update is along shortly |
| Store-and-forward flushes (automatic) | Status that is re-sent anyway |

An ack per reading is traffic the bus does not need. An unacknowledged setpoint
is a fault you will not find until the plant misbehaves.

## Retries and timeouts

An operation gets `BEEBUS_ROTATIONS_OPERATION` (16 rotations, the default for
`BeeBusConfig::rotationsOperation`) to complete: its
request has to reach the destination on one token and the answer come back on
another.

**Expressed in rotations, not milliseconds.** A fixed number of milliseconds
would be either pointless on a slow ring or needlessly impatient on a fast one.

On expiry it is retried, up to `BeeBusConfig::maxRetries` (default **2**). Each
retry waits out one rotation of backoff first, because a rotation is how long it
takes for anything on this network to have had a chance to happen.

When the retries are spent, the operation is dropped and its completion handler
is called with `Timeout`. A confirmed write also raises `OperationTimeout`
through the error handler.

**A dropped operation is always reported.** Dropped silently, a lost setpoint
looks exactly like a delivered one.

## When a node leaves

Everything outstanding for it is concluded with `NodeLost` and dropped — it
cannot now be delivered.

If failsafe is enabled, any read buffer that node was going to fill is cleared
first. A stale reading presented as current is worse than an obviously empty one.

The node's subscriptions to *your* topics go with it, and so does what it told
you about its own. **Your own bindings stay.** A peer coming back should find its
topics still bound, with their quality saying why the values cannot be acted on
meanwhile.

Outputs the lost node had written are driven to their stated safe values. That
raises `onFailsafeApplied` and **not** the register change handler — see
[08 - API reference](08-api-reference.md#failsafe). The two report different
events and an application that cares about the value moving, whatever moved it,
has to register both.

## Completion handlers may queue more work

A completion handler runs after its operation has already left the queue, so it
is free to do anything the API offers — issue the next read in a chain, reissue
a failed write to a standby node, cancel the rest of a batch. The same holds for
the change, topic and membership handlers: nothing the library is still walking
is left exposed to them.

This is a guarantee worth having in writing, because the natural shape of a
control application leans on it:

```cpp
beebus.onWriteComplete([&](uint8_t node, uint16_t addr, BeeBusResult result) {
    if (result != BeeBusResult::Immediate) {
        beebus.writeHreg(standbyNode, addr, value, BeeBusDelivery::Confirmed);
    }
});
```

## Matching responses

Three mechanisms, because there are three kinds of answer.

**Reads** match on destination, function code and address, plus a quantity check.
This is why BeeBus's read responses carry the start address — several reads can
be outstanding to one node at once, and without it a response could not be
attributed.

Bit reads are a special case: a response carries a *byte* count, not a bit count,
so a 1-coil read and an 8-coil read both return one byte. The match is on the
byte count the request would have produced.

**Confirmed writes** match on the operation ID, for the reason above.

**Topic operations** match on destination, topic ID and opcode, kept apart from
the Modbus matcher — which already carries three special cases of its own.

## Publish scheduling

Publishing gets its own path, because it happens continuously rather than on
demand.

On every `loop()` the node examines **32 local topics** with a rotating cursor,
so the whole map is swept every `topicCount() / 32` calls. For each subscriber of
each topic it asks whether a publish is due (deadband moved, interval elapsed,
or never sent) and queues one if so.

Four things keep this bounded:

- **The scan is a fixed slice**, so 3000 topics do not make `loop()` slow. They
  make the sweep take longer, which is exactly the right trade.
- **One publish per (topic, destination) in flight at a time.** Without that, a
  value changing every `loop()` would fill the queue between two tokens.
- **A window per peer for RELIABLE values** (64 by default,
  `BeeBusConfig::streamWindow`): past it a change stays due and coalesces with
  the next.
- **The whole scan is skipped** when no local topic has a subscriber and nothing
  is buffered.

Buffered topics take a different branch: they record every change into their ring
and are published from there and nowhere else. See
[05 - The topic PDU](05-topic-pdu.md).

## Ordering guarantees

What BeeBus promises, and what it does not:

**Promised**

- Operations to one destination are transmitted in queue order.
- Confirmed writes to one topic of one destination are applied in the order they
  were made: a later one waits while an earlier one awaits its acknowledgement.
- On a RELIABLE subscription, the subscriber ends holding the owner's latest
  value whatever was lost on the way.
- All sections of one frame are applied in order at the receiver.
- Store-and-forward samples arrive in the order they were recorded.
- Register writes are applied atomically per request — a change handler never
  sees a half-written block.

**Not promised**

- Ordering between operations to *different* destinations.
- That an unconfirmed (LIVE) operation arrived at all.
- That two topics published in the same turn are applied in the same order at
  every subscriber.

If two values must be consistent with each other, put them in one operation — a
block register write, or a single topic carrying a packed value.

## Reading the queue

```cpp
uint16_t pending  = beebus.getPendingOpCount();
bb_time_t rotation = beebus.getRotationTime();
BeeBusIOStats io   = beebus.getLinkStatistics();
```

| Symptom | Likely cause |
| --- | --- |
| `getPendingOpCount()` near the queue depth | Generating faster than the token rotates. Loosen deadbands, batch writes. |
| `QueueFull` returns | Same, now visible at the call site. |
| Frequent `Timeout` | Ring slower than it looks, or a peer not calling `loop()`. Check `getRotationTime()` and `getHopTime()`. |
| `bufferOverflows` rising | The application is not calling `loop()` often enough. |
| Many `Superseded` | Normal for a fast control loop. Only a problem if you needed every value — in which case buffer the topic. |
| `crcErrors` rising | Collisions, reflections, termination, or a marginal link. |
</content>
