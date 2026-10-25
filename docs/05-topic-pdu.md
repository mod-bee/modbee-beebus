# 05 - The topic PDU

The native data model. Typed values, publish and subscribe, real data quality,
and store-and-forward for nodes that come and go.

Sections carrying these PDUs have **PDU type 1** in the section header.

## What a topic is

A topic is a variable on the node that owns it, addressed on the wire as
**(node ID, topic ID)**.

The frame already carries the node, so topic IDs only have to be unique within
a node. Node 1 topic 1 and node 2 topic 1 are different topics and can be
different types.

| | |
| --- | --- |
| Topic ID range | 1 – 32767 |
| Topic ID 0 | Reserved, never usable |
| Name length | Up to 255 characters, all the length byte in front of it can say |

**Why zero is reserved.** `0` is the null pointer constant, so a literal `0`
would be ambiguous between the ID form and the name form of every single call.
Reserving it also gives the wire a "no topic" value, exactly as node ID 0 means
"no node".

### Names versus IDs

**Names never travel in the data path.** They appear on the wire only in the
commissioning exchange (`LIST_REPLY` and `REGISTER`). Sending 11 characters with
every reading would be indefensible on a radio link.

You state the pairing once, where the topic is bound, and work in names for the
rest of the program:

```cpp
beebus.addInputTopic("pressure", 1, &pressure);
// ... everywhere else
beebus.publishAll("pressure");
```

Setting `BEEBUS_TOPIC_NAMES=0` compiles names out entirely — the pointer per
topic, the lookups, the list exchange and wildcards all disappear, and every
call has an ID-only form beside it that still works.

## Value types

Deduced from the bound C++ variable at compile time. One byte travels with every
value, and the receiver checks it against its own binding.

| Wire | C++ type | Bytes |
| --- | --- | --- |
| `0` | `bool` | 1 |
| `1` | `int8_t` | 1 |
| `2` | `uint8_t` | 1 |
| `3` | `int16_t` | 2 |
| `4` | `uint16_t` | 2 |
| `5` | `int32_t` | 4 |
| `6` | `uint32_t` | 4 |
| `7` | `float` (IEEE 754) | 4 |
| `8` | text, bound as a `char` buffer (see [Text](#text)) | 1 + length |
| `9` | `int64_t` | 8 |
| `10` | `uint64_t` | 8 |
| `11` | `double` (IEEE 754 binary64) | 8 |

Multi-byte values are big-endian. A node that does not know a type code refuses
the value as a malformed PDU and carries on. In IEC 61131-3 terms `9` is `LINT` (and `LTIME`, signed
nanoseconds), `10` is `ULINT` and `LWORD`, `11` is `LREAL`.

**Names** travel behind their own length byte, so a name is up to 255
characters (`BEEBUS_TOPIC_NAME_MAX`, 256 with the terminator). A node keeps its
own names where the application put them and a peer's at their own length, so
a long name costs only where it is used: its own length plus one byte on the
wire. A name too long for one list page goes on a page of its own.

**`int` is deliberately absent.** It is 16 bits on AVR and 32 on ESP32, so
binding one would put a different type on the wire depending on the board.
Attempting it fails to compile with a message naming the types that are allowed.

Bit `0x80` on the type byte is the **aged** flag: a 24-bit age in seconds
follows the value. Bit `0x40` is the **RELIABLE** flag (see
[QoS](#qos-live-and-reliable)). The low six bits (`0x00`–`0x3F`) are the type:
`0`–`11` are in use, the rest are reserved.

### Type mismatches are refused

If a value arrives whose type differs from the binding at the receiving end, it
is refused with `BEEBUS_TOP_ERR_TYPE` and an `ERROR` PDU goes back. The variable
is not written.

This is the whole point of carrying the type. Two ends disagreeing would
otherwise reinterpret four bytes of float as an integer, silently, for ever.

## Text

Bind a character buffer and a topic carries text — a fault message, an
identifier, a state name.

```cpp
char fault[48] = "";

beebus.addInputTopic("fault", 20, fault, sizeof(fault));
strcpy(fault, "pump 2 overload");     // that is the whole publisher
```

**The length travels with the value.** That is the entire reason this is a topic
type rather than a run of registers: nothing at the receiving end has to know
how long the writer's message is. Compare with the register string API, where
`readString(node, addr, buf, size)` requires you to know the size of the *far
end's* buffer, and nothing on the wire tells you.

The buffer is yours, bound by its address and capacity exactly like a string
register (`addString`), and it must outlive the binding. Up to
**`BEEBUS_TOPIC_STRING_MAX` (255)** characters travel in one value. Text
arriving from a peer is cut to the buffer's capacity less one and is always
terminated; the node allocates nothing for it. A bound value longer than 255
characters is carried as far as it goes rather than not arriving at all;
`writeTopic()` refuses one outright, because there the caller can be told.

> **Bind a `char` array, not an Arduino `String`.** A `String` reaches any call
> that takes text as `myString.c_str()`.

Everything else behaves as any other topic:

```cpp
char seenFault[48] = "";

beebus.registerTopic(PUMP, "fault", 20, seenFault, sizeof(seenFault));
beebus.subscribe(PUMP, "fault", 0.0f, 60000);       // deadband is ignored for text
beebus.writeTopic(PUMP, "command", "reset", BeeBusDelivery::Confirmed);

Serial.println(beebus.topicText(PUMP, "fault"));
BeeBusQuality q = beebus.topicQuality(PUMP, "fault");
```

**Deadband is meaningless for text**, so any difference publishes. The
comparison is exact rather than over a hash, so two different messages can never
be mistaken for one another. The maximum interval works normally, and is what
makes silence meaningful for a fault topic just as for a measurement.

### Reading text in a handler

For a string topic, use the string handler rather than the scalar one:

```cpp
beebus.onTopicStringChanged([](uint8_t node, uint16_t topicID,
                               const char* text, uint16_t length) {
    logFault(node, text);
});
```

This matters more than it looks. On a store-and-forward flush the bound variable
is written only for the *newest* sample, while the handler fires for **every**
buffered entry in order — so a handler that read the variable instead would log
the newest message once per entry and lose the history it was trying to capture.

`onTopicStringWritten` is the same for a peer writing a string topic you own.

## Topic IDs on the wire: varints

Topic IDs are variable-length:

| Value | Encoding | Bytes |
| --- | --- | --- |
| 0 – 127 | `0nnnnnnn` | 1 |
| 128 – 32767 | `1nnnnnnn nnnnnnnn` | 2 |

The short form is **canonical**. The long form of a value below 128 is
*refused*, not merely accepted — otherwise one topic could arrive under two
different byte sequences, and nothing that compares encodings would work.

Keeping your topic IDs under 128 saves a byte per PDU. On a slow radio that is
worth doing.

## Opcodes

| Code | Name | Direction |
| --- | --- | --- |
| `0x00` | `PUBLISH` | Publisher → subscriber, unsolicited (or to the whole ring) |
| `0x01` | `WRITE` | Anybody → the topic's owner |
| `0x02` | `READ` | Anybody → the topic's owner |
| `0x03` | `READ_REPLY` | Owner → the node that asked |
| `0x04` | `SUBSCRIBE` | Subscriber → publisher |
| `0x05` | `SUBACK` | Publisher → subscriber |
| `0x06` | `UNSUBSCRIBE` | Subscriber → publisher |
| `0x07` | `LIST_REQUEST` | Anybody → any node |
| `0x08` | `LIST_REPLY` | Any node → the asker (or unsolicited) |
| `0x09` | `REGISTER` | Any node → any node |
| `0x0A` | `ERROR` | Refusal, in reply to anything |
| `0x0B` | `STREAM_ACK` | Receiver → sender: what it has of the sender's RELIABLE stream |
| `0x0C` | `RESULT` | Owner → writer: what it did with a write that asked |
| `0x0D` | `SESSION` | Any node → the ring: its session, on its first turns |

`0x0B`–`0x0D` make up the reliable topic layer, protocol version 2 (see
[QoS](#qos-live-and-reliable) below).

They are numbered from zero because the section header already states the PDU
family before any of this is read. There is no possibility of confusing `0x02`
here with Modbus function code `0x02`.

`READ`, `SUBSCRIBE` and `LIST_REQUEST` are the three that expect an answer;
they stay queued after transmission until the answer arrives or they time out.
A `RESULT` is a confirmed operation of the owner's, acknowledged like a
confirmed write.

A `PUBLISH` may be addressed to the broadcast destination — `DEST` 255, outside
the node range — in which case every node reads the section and whoever holds
the topic takes the value. That is what `publishAll()` sends, and it is the only
opcode that uses it: nothing acknowledges a broadcast, which suits a value being
reported and not a register being written. `Confirmed` is refused with
`InvalidArgument` for the same reason; a value that must be acknowledged goes to
one node with `publishTo()`.

A broadcast carries no stream. A sequence number belongs to one publisher
counting to one destination, and a broadcast has no destination of its own, so
the receiver neither measures it against a stream nor records it into one. A
topic may therefore be broadcast *and* published to its subscribers on their own
terms, and neither disturbs the other. Subscribers count as having received a
broadcast value, so it is not sent to each of them again behind it.

## PDU layouts

Every layout below is exact. Parsing is **strict in both directions**: a PDU
that runs out of bytes is refused, and so is one with bytes left over. Trailing
bytes mean the two ends disagree about the layout, and guessing would be worse
than refusing.

### PUBLISH (0x00) and READ_REPLY (0x03)

```
┌──────┬──────────────┬─────┬──────┬───────────┬───────────────┐
│ 0x00 │ topic varint │ seq │ type │  value    │ age24 (opt)   │
│  1   │    1 or 2    │  1  │  1   │  1/2/4/8  │      3        │
└──────┴──────────────┴─────┴──────┴───────────┴───────────────┘

a string value being length-prefixed instead of fixed width:

┌──────┬──────────────┬─────┬──────┬─────┬──────────┬─────────────┐
│ 0x00 │ topic varint │ seq │ 0x08 │ len │ bytes... │ age24 (opt) │
│  1   │    1 or 2    │  1  │  1   │  1  │   len    │      3      │
└──────┴──────────────┴─────┴──────┴─────┴──────────┴─────────────┘
```

The age is present only when bit `0x80` of the type byte is set. It is
big-endian **seconds**, giving a range of 194 days — deliberately outliving
`bb_millis()`'s 49.7-day wrap, so the wire format never becomes the limit.

Bit `0x40` of the type byte is **RELIABLE**: the `seq` is then the sender's
stream to this receiver (one count for all its topics, see
[QoS](#qos-live-and-reliable)), not the topic's own. The type itself is the low
six bits.

Ages under one second are not sent; three bytes is not worth it.

**Why they are separate opcodes despite an identical layout:** a solicited reply
must update the sequence counter *without* raising a gap event, while an
unsolicited publish must raise one. A reply is not part of the stream's cadence.

### WRITE (0x01)

```
┌──────┬──────────────┬───────┬──────────────────────────────────────────┬──────┬───────────┐
│ 0x01 │ topic varint │ flags │ request16 session16 ownerSession16 (opt) │ type │  value    │
│  1   │    1 or 2    │   1   │                    6                     │  1   │  1/2/4/8  │
└──────┴──────────────┴───────┴──────────────────────────────────────────┴──────┴───────────┘
```

| Flag | Meaning |
| --- | --- |
| `0x01` REQUEST | A request ID, the writer's session and the owner's session as the writer knows it (0 unknown) follow. The owner applies each ID once, and refuses (`0x07` stale) a request made to an earlier instance of itself. Set on every confirmed write. |
| `0x02` RESULT | The owner answers with a `RESULT` once it has decided. Only with REQUEST. |

A plain (LIVE) write has flags `0` and nothing else extra. The request ID is
given as the write goes into a frame, per writer → owner pair, and kept for its
retries, so a write whose acknowledgement was lost is acknowledged again and
**not applied again**. A writer that hears no
decision asks again under the same ID (twice at most, waiting for an owner that
left the ring); the owner answers a repeat from the decisions it remembers (16).
A request belongs to the sessions it was made in: it is never asked again
under a new session of the writer's, and an owner that has restarted refuses
it, so the writer is told "no answer" rather than have it applied twice.

When the sender asks for confirmation, the section's `ACK` bit is set and its
operation ID follows the PDU — that lives in the section header, not here. See
[04 - Frame format](04-frame-format.md).

### READ (0x02) and UNSUBSCRIBE (0x06)

```
┌──────┬──────────────┐
│ 0x02 │ topic varint │
└──────┴──────────────┘
```

### SUBSCRIBE (0x04)

```
┌──────┬──────────────┬────────────────┬──────────────┬─────┬───────────┐
│ 0x04 │ topic varint │ deadband f32   │ interval16   │ qos │ session16 │
│  1   │    1 or 2    │       4        │      2       │  1  │     2     │
└──────┴──────────────┴────────────────┴──────────────┴─────┴───────────┘
```

`qos` bit `0x01` is RELIABLE. The session is the subscriber's: a new one is a
new subscriber, and what the old one held is dropped first.

The deadband is **always** `float32`, whatever the topic's own type. A threshold
in engineering units is a real number, and one comparison path then serves every
type. For an 8-byte type the owner takes the difference in `double`, so a move
of 0.5 in a value of 10^12 is still seen.

### SUBACK (0x05)

```
refused:   ┌──────┬──────────────┬────────┬───────────┐
           │ 0x05 │ topic varint │ status │ session16 │
           └──────┴──────────────┴────────┴───────────┘
accepted:  ┌──────┬──────────────┬──────┬───────────┬──────┬─────┬──────┬───────┬─────────────┐
           │ 0x05 │ topic varint │ 0x00 │ session16 │ 0x01 │ seq │ type │ value │ age24 (opt) │
           └──────┴──────────────┴──────┴───────────┴──────┴─────┴──────┴───────┴─────────────┘
buffered:  ┌──────┬──────────────┬──────┬───────────┬──────┬──────┐
           │ 0x05 │ topic varint │ 0x00 │ session16 │ 0x00 │ type │
           └──────┴──────────────┴──────┴───────────┴──────┴──────┘
```

A `SUBACK` travels **whatever the outcome**, carries the owner's session, and
an acceptance carries the topic's **current value** (type byte and value as in
`PUBLISH`, RELIABLE-numbered for a RELIABLE subscription). The first value
therefore cannot be lost on its own: a lost `SUBACK` is asked for again by the
subscriber's retry, so one lost frame cannot leave a setting blank. While a
`SUBACK` is still queued, a newer value
of its topic is written into it rather than sent separately: answers leave
after operations in a frame, so a separate publish would arrive first and the
older answer after it.

A buffered topic's answer carries only the type: its values come out of its
ring, in order. The type is what lets the two ends agree on the binding before
a value moves: a subscription that would land in a variable of the wrong type
is reported as `TypeMismatch` at once.

### LIST_REQUEST (0x07)

```
┌──────┬───────────────┬───────┐
│ 0x07 │ cursor varint │ limit │
└──────┴───────────────┴───────┘
```

`limit` is the maximum entries wanted, 1–127, with **bit 7 meaning "include
names"**.

Two flavours matter. IDs and types alone cost about 4 bytes an entry; names
included cost 15–30. Fetching a few thousand named topics over LoRa is minutes
of airtime, so the choice is yours to make.

### LIST_REPLY (0x08)

```
┌──────┬───────┬───────┬────────────────────┬────────────┐
│ 0x08 │ flags │ count │ nextCursor varint  │ entries... │
└──────┴───────┴───────┴────────────────────┴────────────┘

flags: bit 0 = more pages follow
       bit 1 = entries carry names

entry: ┌──────────────┬──────┬───────┬─────────┬────────┐
       │ topic varint │ type │ flags │ namelen │ name   │
       └──────────────┴──────┴───────┴─────────┴────────┘
                                      (only when bit 1 is set)
```

**The cursor is a topic ID, not an index.** Topics added or removed between
pages therefore cannot make the walk skip or repeat one.

A page is capped at **200 bytes or 32 entries**, whichever comes first, so a
list reply still leaves room in the frame for other sections. Its first entry
always goes, so an entry with a name longer than a page travels on a page of
its own.

Pull and push share this opcode pair: an *unsolicited* `LIST_REPLY` is the push
form, which is what `publishTopicList()` sends.

### REGISTER (0x09)

```
┌──────┬──────────────┬──────┬───────┬─────────┬────────┐
│ 0x09 │ topic varint │ type │ flags │ namelen │ name   │
└──────┴──────────────┴──────┴───────┴─────────┴────────┘
```

One topic announced to one peer. `registerTopicWith()` sends it.

### ERROR (0x0A)

```
┌──────┬──────────────┬────────┐
│ 0x0A │ topic varint │ reason │
└──────┴──────────────┴────────┘
```

### RESULT (0x0C)

```
┌──────┬──────────────┬───────────┬─────────┬──────┬───────────┐
│ 0x0C │ topic varint │ request16 │ outcome │ type │  value    │
└──────┴──────────────┴───────────┴─────────┴──────┴───────────┘
```

What the owner did with request `request16`, and the value it holds. The value
is refreshed as the `RESULT` goes into a frame (retries included), so it is
always the newest word from the owner on that topic when it arrives.

| Outcome | Meaning |
| --- | --- |
| `0x00` ACCEPTED | Stored as sent (a command: taken) |
| `0x01` CHANGED | The owner kept another value (clamped, rounded, put back), carried here |
| `0x02` REFUSED | Not taken (out of range, refused by validate); the value kept is carried |
| `0x03` READ_ONLY · `0x04` NO_TOPIC · `0x05` TYPE | Nothing done, and why |
| `0x06` BUSY | No room to hold the request for a decision: ask again |

### STREAM_ACK (0x0B)

```
┌──────┬─────────┬───────┬────────────────────┐
│ 0x0B │ highest │ flags │ missing (0..8 B)   │
└──────┴─────────┴───────┴────────────────────┘
```

`highest` is the highest RELIABLE stream number received from the section's
destination; the low four bits of `flags` say how many bytes of `missing`
follow, low byte first: bit *i* set means number `highest - 1 - i` never
arrived. Three bytes when nothing is missing.

### SESSION (0x0D)

```
┌──────┬───────────┐
│ 0x0D │ session16 │
└──────┴───────────┘
```

Broadcast (`DEST` 255) on a node's first three token turns after it starts or
drops its view of the ring.

### Per-topic flags

Carried in `LIST_REPLY` and `REGISTER`:

| Bit | Meaning |
| --- | --- |
| `0x01` | Writable — peers may write it |
| `0x02` | Buffered — publishes go through a store-and-forward ring |

### Refusal reasons

Carried in `ERROR` and `SUBACK`:

| Code | Meaning |
| --- | --- |
| `0x00` | OK |
| `0x01` | No such topic — nothing bound at that ID |
| `0x02` | Type mismatch |
| `0x03` | Read only — bound, but peers may not write it |
| `0x04` | Malformed — the PDU did not parse |
| `0x05` | No room — no subscriber slot or contract left |
| `0x06` | Refused — a served topic's range or validate refused the value |
| `0x07` | Stale — a request made to an earlier instance (session) of this node |

**Every refusal travels back.** Nothing is left to time out when the reason is
already known, and the reason is more useful than a Modbus exception code could
express.

## Subscriptions

A subscription states two things, and both belong to the *subscription*, not the
topic:

```cpp
beebus.subscribe(PUMP, "pressure", 0.5f, 60000);
//                                  │      └─ maximum interval, ms
//                                  └──────── deadband, in the value's units
```

**Two subscribers may hold entirely different terms for the same topic.** A
local display asks for ±0.1 bar; a supervisor five kilometres away over a radio
asks for ±2 bar and once every ten minutes. Both get exactly what they asked
for. A single per-topic setting could not honour both, and publishing on the
tightest one would flood the slow subscriber — which is precisely what a
deadband exists to prevent.

### The interval encoding

`interval16` on the wire:

| Bit 15 | Unit | Range |
| --- | --- | --- |
| clear | deciseconds | 0.1 s – 54 min |
| set | seconds | 1 s – 9.1 h |

`0` means **report by exception**: no periodic obligation at all.

Encoding always rounds **up**, so a decoded interval never promises sooner than
what was asked for. The subscriber records the interval as it will actually
travel, so quality is judged against what the publisher was really asked for
rather than against an unrounded request.

### When a publish is due

On every `loop()`, the publisher examines a slice of its topics (32 per call,
with a rotating cursor, so the whole map is swept every `topics / 32` calls) and
asks, for each subscriber:

1. Has this subscriber ever been sent anything? If not, it is owed a report
   **now**, whatever the deadband. A new subscriber gets the current value
   immediately — in its `SUBACK` — and this is what makes every topic
   inherently retained. A RELIABLE value the receiver reported missing is
   owed again the same way.
2. Has the maximum interval elapsed since the last send? If so, due.
3. Is the value unchanged from what was last sent? If so, not due.
4. Is the deadband zero? If so, any change is due.
5. Otherwise: has the value moved by at least the deadband?

Note that step 3 compares **bits, not floats**. A float comparison would lose a
one-bit change in a 32-bit integer.

Only one publish of a given topic to a given destination is allowed in flight at
a time. Without that, a value changing every `loop()` would fill the operation
queue between two tokens.

### Subscriptions and restarts

Subscriptions live on the **publisher**. A node that restarts has lost its
subscriber list.

BeeBus handles this: when a peer rejoins the ring, every subscription this node
holds to it is re-issued automatically. They are queued gently — a few per
token — because a peer may carry more subscriptions than the operation queue
holds. A subscription that finds the queue full is held the same way and sent
as room frees, so an application may subscribe everything at once.

A peer that restarts **before the ring noticed it leave** is told apart by its
session (see [Sessions](#sessions)): its subscriptions are dropped and ours to
it are re-issued, whether or not this node heard the ring's announcements.

Placing your subscriptions inside `onMemberJoined` is still the right habit: it
covers the first join and gives you one obvious place for `fetchTopics` too.

## Sequence numbers and gaps

**Every publish carries a one-byte sequence number, and it is per (topic,
destination) pair.**

Per destination, not per topic. Subscriber A with a tight deadband and
subscriber B with a loose one receive different subsets of the same topic's
changes; a single counter would look like a stream full of holes to B.

The number is stamped at **send time**, from the destination's own slot, and
only once the operation has been accepted into the queue — so a refused publish
does not consume a number and open a gap that never happened.

### The gap rule

On arrival, `delta = (uint8_t)(seq - lastSeq)`:

| delta | Meaning | Action |
| --- | --- | --- |
| `0` | Duplicate | Accept, no event |
| `1` | In order | Normal |
| `2`–`127` | **Gap of `delta - 1`** | Raise `onTopicGap` (only for `PUBLISH`) |
| `128`–`255` | Stale | Discard |

A stream that has not been seen before is measured from zero, because a
publisher numbers each destination from there. The first publish is therefore
delta 1 and reports nothing.

The stale test only applies **once a stream is established**. When a publisher
leaves the ring, the receiver clears its record of the stream, so a restarted
publisher numbering from zero again is not mistaken for a stale duplicate.

### Why this matters

This is what makes event-driven telemetry honest.

Without a sequence number, a value that has not changed and a publisher that has
stopped being heard look **identical** at the receiving end. Every pure
publish-on-change protocol has this problem, and most solve it by adding a
heartbeat and hoping.

```cpp
beebus.onTopicGap([](uint8_t node, uint16_t topicID, uint8_t missed) {
    // You now know exactly how many updates went missing, and from where.
});
```

## QoS: LIVE and RELIABLE

Every subscription, publish, write and bind has a QoS. **LIVE** (the default
everywhere) is what the sections above describe: best effort, newest wins, a
deadband allowed. Right for levels, flows and speeds updated every scan.

**RELIABLE** delivers every change: settings, states, alarms, totals,
commands. For a one-off write or `publishTo` it is `Confirmed` (acknowledged,
retried, applied once). For a subscription it is a **stream**:

- Each sender → receiver pair numbers its RELIABLE values (one count for every
  topic between the two, in the type byte's `0x40` bit and the `seq` byte). The
  number is given as the section goes into a frame, in frame order.
- The receiver says what it has in its own next frame: one `STREAM_ACK` per
  sender, rewritten while it waits for the token. Nothing extra is sent while
  nothing new arrives.
- A number the receiver reports missing makes that topic due again, and it goes
  as its **current value**: an old value is never retried, only the latest. The
  mask is repeated in every acknowledgement until the number is 64 behind, so a
  lost acknowledgement does not lose a gap.
- No acknowledgement progress for `rotationsStreamResend` rotations (default 4)
  while something is awaited: everything unconfirmed goes again, once.
- A **window** per pair (`streamWindow`, default 64): when it is full, changes
  stay due and coalesce into the latest value. A topic changing every `loop()`
  costs one value in flight, not a queue.
- A value equal to what the receiver holds (a repair crossing its original) is
  not applied or raised a second time.

There is no periodic refresh: the ring is the heartbeat, and a stream is either
in sync or knows why not. A buffered flush and a read reply keep the topic's own
count on a RELIABLE subscription, as they are confirmed one by one already.

**Cost, measured** (two nodes, 50 settings of which 5 change every 200 ms,
115200 baud): 21 bytes per rotation LIVE, 23 RELIABLE; quiet, 15
both. On a 5 % loss line every RELIABLE value ends right; LIVE makes no such
promise.

## Sessions

Each node draws a 16-bit session at `begin()` (the platform's random source;
`BeeBusConfig::sessionId` pins it) and a new one whenever it drops its whole
view of the ring. It travels in `SUBSCRIBE`, `SUBACK`, every write request, and
a `SESSION` broadcast on the node's first three turns. A peer seeing a session
it knows under another value treats the node as restarted: its slots, both
streams and its request window are dropped, the subscriptions held to it are
re-issued, and binds to it go `SYNCING`. Sparkplug's `bdSeq` does the same job.

## Write results

A write with `wantResult` (a REQUEST with the RESULT flag) is answered by the
owner with what it did:

- A **served** topic with a range refuses outside it; with a `validate` it
  decides at once (`ACCEPTED`, `CHANGED` with the value it keeps, `REFUSED`).
- Otherwise the owner stores the value and **reads it back** after its next
  `loop()` and the topic's `settleMs`: the program runs in between, so a block
  that refuses a value and puts the old one back answers `CHANGED`. A queued
  topic is read back from when the program takes the write (`topicApplied`).
- A **command** (a served topic of kind `Command`) answers `ACCEPTED` once
  taken: an event, applied exactly once.
- Sixteen writes can wait for a decision at once; a seventeenth is answered
  `BUSY` and not applied.

## Bidirectional binding: serve and bind

Built only from the pieces above, for applications that want one variable kept
the same on both sides. One node **serves** a topic (the owner decides); any
node **binds** it: a subscription (LIVE by default like everything else;
RELIABLE for a setpoint, a setting or configuration) and writes that ask for a
result.
Change the bound variable and the change goes to the owner; the owner's value
comes back into the variable whatever it decided. States (`BeeBusBindState`):
`Offline`, `Syncing`, `InSync`, `Pending`, `Refused` (the variable holds the
kept value), `Failed` (no decision came back). No timers, retries or pacing in the application. See
[08 - API reference](08-api-reference.md#bidirectional-binding).

## Data quality

```cpp
BeeBusQuality q = beebus.topicQuality(PUMP, "pressure");
```

| Value | Meaning |
| --- | --- |
| `Unknown` | Never received |
| `Good` | Within the contract, publisher in the ring |
| `Uncertain` | Late, but within twice the contract |
| `Bad` | Beyond that, or the publisher has left the ring |

The rule, where *T* is the maximum interval the subscription asked for and *A*
is the age of the value:

```
never received              → Unknown
publisher not in the ring   → Bad
T == 0 (report by exception)→ Good     (nothing was promised; membership is the answer)
A ≤ T                       → Good
T < A ≤ 2T                  → Uncertain
A > 2T                      → Bad
```

**Quality comes from the subscription contract, not from a constant.** This is
worth dwelling on. A library that hard-coded "stale after 5 seconds" would be
wrong for a temperature sampled hourly and wrong for a pressure sampled ten
times a second. Here, *you* stated what you would accept when you subscribed,
and the library holds the publisher to exactly that.

If you subscribe with an interval of 0, you have promised nothing, so quality
reduces to "is the publisher still in the ring". That is a legitimate choice for
an event, and a poor one for a measurement. On a RELIABLE subscription it is
also an honest one for a setting: a value lost on the way is repaired within a
rotation or two, so `Good` means the owner's latest value.

```cpp
bb_time_t age = beebus.topicAge(PUMP, "pressure");   // ms, or BEEBUS_SILENCE_NEVER
```

## Store and forward

For a node that cannot always reach the network — a battery sensor that wakes,
samples and sleeps; a node on a radio link that fades.

```cpp
beebus.bufferTopic("pressure", 20);      // Hold up to 20 samples
```

Depth is capped at **127**, and `0` turns it off.

**Why 127 and not more:** a one-byte sequence with a "delta over 127 means
stale" rule cannot express a longer flush. A 200-deep buffer would have its tail
silently discarded by the receiver, which is worse than not offering it.

### How it behaves

**Every publish of a buffered topic goes through the ring — network up or
down.** There is one path and no fast case.

If a live publish could jump the queue, then after a reconnect the receiver
would see sequence *n+40*, then *n*, *n+1*, … read each of those as a backward
jump, and discard the entire history. One conditional in the wrong place would
destroy the feature, so the structure prevents it.

When the ring is full, the newest sample overwrites the oldest, and the loss is
*counted*. On the next flush those counts are **spent as sequence numbers**, so
the receiver sees a gap of exactly that size and raises `onTopicGap`. Nothing is
lost silently. (A loss over 126 reports as 126, since 128 would read as stale.)

### Draining

Buffered samples are released while the node holds the token, immediately before
the frame is built, so they ride *this* turn rather than waiting a whole rotation.

Four gates, all required:

1. This node is taking part in a ring.
2. At least **4 rotations** since joining (`rotationsFlushSettle`) — a sensor that has just woken is
   joining a ring whose membership is still settling, and a history poured into
   that is a history poured away.
3. The operation queue is under half full, so live traffic always gets through.
4. The topic has at least one subscriber that is currently a ring member.

At most **4 samples per token**, and room is checked for every destination
before any is sent, so a sample is never delivered to half the subscribers and
repeated to the rest under a different sequence number.

Flushed samples are sent **Confirmed**. Spending RAM to keep a history and then
dropping it into the dark would not be coherent.

### At the receiving end

The age is used to **back-date** the arrival time (`rxAt = now - age`) rather
than being stored beside the value. Quality, freshness and newest-wins then all
work unchanged, at zero extra cost per record.

The bound variable is written only when the sample is **newer** than what is
already held — a flush arrives oldest first, so without that the variable would
end up holding the *oldest* sample. Every entry still reaches `onTopicChanged`
in arrival order, so a handler that is logging gets the whole history in the
right order while the variable holds the truth.

```cpp
uint16_t waiting = beebus.bufferedCount("pressure");
```

## Commissioning

For discovering what a node has, rather than knowing in advance.

```cpp
beebus.fetchTopics(PEER, true);            // Walk the peer's whole list, with names
beebus.publishTopicList(PEER, true);       // Push ours the other way
beebus.registerTopicWith(PEER, "pressure");// Announce one topic
```

`fetchTopics` walks the entire list one page per token, chaining the pages
automatically, and reports each entry through `onTopicList`:

```cpp
beebus.onTopicList([](uint8_t node, const BeeBusTopicEntry& entry) {
    Serial.print(node);       Serial.print(' ');
    Serial.print(entry.topicID); Serial.print(' ');
    Serial.print(entry.name); Serial.print(' ');
    Serial.println(bb_topicTypeName(entry.type));
});
```

A fetch without names does not erase names an earlier fetch learnt.

### Wildcards

Once a fetch has completed, names can be matched:

```cpp
beebus.subscribeAll(PEER, "temp*", 0.5f, 60000);
```

Globs, not MQTT wildcards: `*` is any run of characters, `?` is exactly one.
Names are flat, with no level separators, so promising MQTT's `+` and `#` — which
mean specific things about `/`-separated levels — would be a lie.

`subscribeAll` requires a completed `fetchTopics` for that node and returns
`NotMapped` without one. Chaining an unbounded asynchronous operation behind a
synchronous-looking call would hide it. A match still needs a variable bound to
land in, so unregistered matches are passed over rather than half subscribed.

The expansion is paced over the following tokens rather than queued at once —
200 matches against a 50-slot queue would not end well.

## Memory

| Record | 32-bit target | 64-bit host |
| --- | --- | --- |
| Local topic | 44 B | 64 B |
| Remote topic | 28 B | 40 B |
| Subscriber slot | 28 B | 28 B |
| Contract (interned) | 12 B | 12 B |
| Buffered sample | 16 B | 16 B |
| Peer (streams, session, request window) | 32 B | 32 B |
| Write held for a decision (16, taken at `begin()`) | 32 B | 32 B |
| Served topic | — | 72 B |
| Bind (plus 8 B shadow, or twice the text capacity) | — | 88 B |

Sizes are with `BEEBUS_TOPIC_NAMES` on. A text topic also holds a copy of its
last text, the size of its buffer. A peer record exists per node ever heard
from.

Local and remote topics are stored **apart**, because their fields are almost
disjoint and a gateway's 3000 remote topics should not each carry a subscriber
list head they will never use.

They are **sorted vectors**, not `std::map`: lookup is a binary search, insert is
a memmove. At 3000 topics a single fat `std::map` measures around 144 KB and the
role-split sorted vectors around 49 KB. On a 300 KB ESP32 heap that difference
is the feature shipping or not.

Consequences worth knowing:

- **Insert is O(n).** Use `reserveTopics(n)` if you are adding a lot, and add
  them in ascending ID order for the fast path.
- **Growth doubles up to 256 entries, then goes linear**, so a large map never
  needs half its own size again in free memory to grow once.
- **Never hold a pointer to a topic record across an add.** The API returns
  indices and copies for exactly this reason.

Subscription terms are **interned**: subscribers wanting the same deadband and
interval share one 12-byte record. A gateway with 3000 topics typically uses
three. The limit is 255 distinct sets of terms.

```cpp
beebus.reserveTopics(500, 2000);    // local, remote
```

## Next

- [06 - The Modbus PDU](06-modbus-pdu.md) — the other family.
- [08 - API reference](08-api-reference.md) — every topic call.
- [12 - Best practices](12-best-practices.md) — choosing deadbands and intervals.
</content>
