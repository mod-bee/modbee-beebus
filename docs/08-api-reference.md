# 08 - API reference

Every public call on `BeeBusAPI`, what it returns, and when.

Include one header for the library:

```cpp
#include <BeeBusGlobal.h>
```

Transports are deliberately left out of it, so the core carries no platform
dependency. Include the one you use from `transport/` as well — for example
`<transport/BeeBusHostUdpTransport.h>` — or, on Arduino, pass a `Stream*` to
`begin()` and include nothing more ([07 — Transports](07-transports.md)).

## Contents

- [Lifecycle](#lifecycle)
- [Ring membership](#ring-membership)
- [Topics — binding](#topics--binding)
- [Topics — moving data](#topics--moving-data)
- [Topics — subscriptions](#topics--subscriptions)
- [Topics — quality and freshness](#topics--quality-and-freshness)
- [Topics — store and forward](#topics--store-and-forward)
- [Topics — commissioning](#topics--commissioning)
- [Topics — events](#topics--events)
- [Bidirectional binding](#bidirectional-binding)
- [Registers — binding](#registers--binding)
- [Registers — local access](#registers--local-access)
- [Registers — remote access](#registers--remote-access)
- [Strings](#strings)
- [Failsafe](#failsafe)
- [Events and completion](#events-and-completion)
- [Diagnostics](#diagnostics)
- [Return values](#return-values)
- [Enumerations](#enumerations)
- [Free functions](#free-functions)
- [Constants worth knowing](#constants-worth-knowing)

---

## Lifecycle

```cpp
bool begin(BeeBusTransport* transport, uint8_t nodeID = 1,
           const BeeBusConfig& config = BeeBusConfig());
bool begin(Stream* serial, uint8_t nodeID = 1,
           const BeeBusConfig& config = BeeBusConfig(),
           uint32_t baudRate = 0);                            // Arduino only
void loop();
void end();
bool isInitialized();
const char* startError() const;   // why begin() / rebind() failed, when it was memory
```

`begin()` starts the protocol on a transport. The caller keeps ownership of the
transport, which must outlive this instance. Returns **false** if the node ID is
outside 1–254, the transport is null or cannot be opened, `begin()` has
already been called, or the board has no room for the node, its buffers and
queues or (the `Stream*` form) its link — then `startError()` says which:
*"out of memory for the node"*, *"... for the buffers and queues"*,
*"... for the link"*. It is null for any other refusal. Everything a running
node needs is taken here, so it never starts with less; a `rebind()` that
cannot find memory changes nothing.

The config belongs to *this instance*, so several nodes in one process can be
tuned independently.

The `Stream*` overload is an Arduino convenience: it wraps the stream in a
transport this instance owns and deletes. Pass `baudRate` as the same rate the
sketch gave `Serial.begin()`: it is the one property of the line a node cannot
measure for itself, and with it the interframe gap, the response window and a
joining node's announcement are sized from the rate rather than measured. `0`
leaves the node to measure it.

`loop()` drives everything — receive, frame assembly, token handling,
subscriptions, timeouts. **Call it every pass of your main loop.** It never
blocks. At most 1024 bytes are drained per call, so a flooded node still runs
your application.

`end()` shuts the node down and releases everything. Don't call it from inside
a handler: the handler is running on the node it frees.

```cpp
void connect();
void disconnect();
bool isConnected();
uint8_t getNodeID();
bool hasDuplicateNodeID();
uint16_t session();
```

`connect()` starts the join sequence. Bind your data and register your handlers
*before* calling it, because peers can write to this node the instant it joins.

`disconnect()` announces departure — the ring closes over the node cleanly
rather than waiting for it to time out — and clears pending operations.

`isConnected()` means: on a network, and aware of at least one other node.

`hasDuplicateNodeID()` is true while this node has stood down because another
node is transmitting under its ID. Without it a commissioning mistake looks
exactly like a node that has not finished starting: `isConnected()` is false,
`isMember()` finds nobody, and every read and write is refused. It clears once
the address has been quiet long enough, or on the next `connect()`.

`session()` is this instance's session number, which lets peers tell a restart
apart from the instance before it ([05 — Sessions](05-topic-pdu.md#sessions)).

```cpp
bool rebind(BeeBusTransport* transport, uint8_t nodeID,
            const BeeBusConfig* config = nullptr);
bool rebind(Stream* serial, uint8_t nodeID, uint32_t baudRate = 0,
            const BeeBusConfig* config = nullptr);              // Arduino only
bool isLeaving();
```

`rebind()` changes the node ID, the line or the configuration **while running**,
keeping everything the application bound: registers, topics, subscriptions,
failsafes and handlers. What the node knew about its old ring is dropped — it
joins under the new ID from scratch, its peers subscribe again when it arrives,
and its own subscriptions are asked for again as each peer joins. Every
operation still owed concludes `Cancelled` before it returns. Call
`disconnect()` first and wait while `isLeaving()` so the ring hears the node go
(a node outside a working ring stops at once; one whose turn never comes gives
up after the rejoin timeout), then `rebind()`, then `connect()`. Returns
false — and nothing changes — for an ID out of range, a transport that will not
open, or before `begin()`. `config`, when given, replaces the configuration; otherwise
the one in force is kept. A transport `begin(Stream*)` made is freed; one you
passed is still yours. Like `end()`, it must not be called from inside a
handler.

---

## Ring membership

```cpp
bool    isMember(uint8_t nodeID);
uint8_t getMemberCount();
uint8_t getMemberAt(uint8_t index);
bb_time_t silenceFor(uint8_t nodeID);
bool    isAlive(uint8_t nodeID);
```

`isMember()` is true only when **this node is itself taking part in a ring** and
the named node is in it. A node still joining reports nobody as a member.

`getMemberCount()` includes this node. Returns 0 while not taking part.

`getMemberAt()` returns a member by position, or 0 past the end, so the ring can
be walked. The order is the order the token takes, so consecutive entries are
the two ends of one hop:

```cpp
for (uint8_t i = 0; i < beebus.getMemberCount(); i++) {
    const uint8_t node = beebus.getMemberAt(i);
    Serial.print(node);
    Serial.print(F(" hop "));
    Serial.print(beebus.getHopTime(node));
    Serial.print(F(" ms, allowance "));
    Serial.println(beebus.getHopTimeout(node));
}
```

`silenceFor()` is milliseconds since anything was heard from a node, or
`BEEBUS_SILENCE_NEVER` (`0xFFFFFFFF`) if nothing ever has. This is the raw
number behind `isAlive()` and behind topic quality, and it is a good diagnostic
in its own right.

`isAlive()` is `isMember()` and heard from recently enough.

```cpp
void onMemberJoined(BeeBusMemberHandler handler);   // void(uint8_t nodeID)
void onMemberLeft(BeeBusMemberHandler handler);
```

Raised after the membership list has been updated. **`onMemberJoined` is where
subscriptions belong**, because it fires on the first join and on every rejoin.

---

## Topics — binding

### Topics this node owns

```cpp
template <typename T> bool addTopic(const char* name, uint16_t topicID, T* variable);
template <typename T> bool addInputTopic(const char* name, uint16_t topicID, T* variable);

template <typename T> bool addTopic(uint16_t topicID, T* variable);
template <typename T> bool addInputTopic(uint16_t topicID, T* variable);

bool removeTopic(uint16_t topicID);
bool removeTopic(const char* name);
```

`addTopic` binds a topic peers may **read and write**. `addInputTopic` binds one
they may **only read**.

| | |
| --- | --- |
| `name` | Must outlive the binding — pass a string literal. Never copied. |
| `topicID` | 1 – 32767. Zero is reserved and always fails. |
| `variable` | `bool`, `int8_t`, `uint8_t`, `int16_t`, `uint16_t`, `int32_t`, `uint32_t`, `int64_t`, `uint64_t`, `float` or `double` (`double` only where it is 8 bytes). Anything else fails to compile. Text binds a `char` buffer instead; see [Text topics](#text-topics). |

Returns **false** if the pointer is null, the ID is out of range, or the ID is
already bound.

```cpp
float pressure = 0.0f;
beebus.addInputTopic("pressure", 1, &pressure);
```

### Topics belonging to a peer

```cpp
template <typename T>
bool registerTopic(uint8_t nodeID, const char* name, uint16_t topicID, T* variable);

template <typename T>
bool registerTopic(uint8_t nodeID, uint16_t topicID, T* variable);

bool unregisterTopic(uint8_t nodeID, uint16_t topicID);
bool unregisterTopic(uint8_t nodeID, const char* name);
```

Binds one of your variables to a peer's topic. **Required before anything can be
received** — a value arriving for a topic with nothing bound to it is dropped,
because the binding is what states the expected type.

```cpp
float pumpPressure = 0.0f;
beebus.registerTopic(PUMP, "pressure", 1, &pumpPressure);
```

### Text topics

```cpp
bool addTopic(const char* name, uint16_t topicID, char* text, uint16_t capacity);
bool addInputTopic(const char* name, uint16_t topicID, char* text, uint16_t capacity);
bool registerTopic(uint8_t nodeID, const char* name, uint16_t topicID,
                   char* text, uint16_t capacity);

bool addTopic(uint16_t topicID, char* text, uint16_t capacity);
bool addInputTopic(uint16_t topicID, char* text, uint16_t capacity);
bool registerTopic(uint8_t nodeID, uint16_t topicID, char* text, uint16_t capacity);

BeeBusResult writeTopic(uint8_t nodeID, uint16_t topicID, const char* text,
                        BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed,
                        bool wantResult = false);
BeeBusResult writeTopic(uint8_t nodeID, const char* name, const char* text,
                        BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed,
                        bool wantResult = false);

const char* topicText(uint16_t topicID);                 // "" when unset
const char* topicText(uint8_t nodeID, uint16_t topicID);
const char* topicText(const char* name);
const char* topicText(uint8_t nodeID, const char* name);
```

Bind a character buffer and the topic carries text, length-prefixed on the
wire, so nothing at the far end needs to know how long the value is.

```cpp
char fault[32] = "";
beebus.addInputTopic("fault", 20, fault, sizeof(fault));
strcpy(fault, "pump 2 overload");
```

The buffer is yours, bound by its address and capacity like a string register
(`addString`), and must outlive the binding. Text arriving from a peer is cut
to `capacity - 1` characters and always terminated; the node allocates nothing
for it. An Arduino `String` reaches any call taking text as `myString.c_str()`.

Up to `BEEBUS_TOPIC_STRING_MAX` (255) characters travel. A bound value longer
than that is carried as far as it goes; `writeTopic()` refuses one with
`InvalidArgument`, since there the caller can act on it. `topicText()` returns
`""` when the topic is not bound, is not text, or nothing has arrived.

Deadband is ignored for text — any difference publishes, compared exactly. The
maximum interval applies normally.

### Capacity and names

```cpp
void reserveTopics(uint16_t localCount, uint16_t remoteCount = 0);

uint16_t topicIdOf(const char* name);
uint16_t remoteTopicIdOf(uint8_t nodeID, const char* name);

bool rebindTopic(uint16_t topicID, void* variable);
bool rebindRemoteTopic(uint8_t nodeID, uint16_t topicID, void* variable);
```

`reserveTopics()` matters when you are adding many topics: each insert keeps the
map sorted, so adding thousands one at a time is thousands of moves.

`rebindTopic()` and `rebindRemoteTopic()` point an already-bound topic at
different storage, for a caller whose variable has moved. The type and the name
are unchanged.

The `*IdOf` calls return `BEEBUS_TOPIC_NONE` (`0xFFFF`) if the name resolves to
nothing. A remote name resolves against your own bindings first, then against
whatever `fetchTopics` discovered.

Every named call has an ID-only form beside it, so a build with
`BEEBUS_TOPIC_NAMES=0` still has a complete API.

### Type given at runtime

```cpp
bool addTopicRaw(uint16_t topicID, uint8_t type, void* variable, bool writable,
                 const char* name, uint16_t textCapacity = 0);
bool registerTopicRaw(uint8_t nodeID, uint16_t topicID, uint8_t type, void* variable,
                      const char* name, uint16_t textCapacity = 0);
BeeBusResult writeTopicRaw(uint8_t nodeID, uint16_t topicID, const BeeBusScalar& value,
                           BeeBusDelivery delivery, bool wantResult = false);
```

The same work the templates do, for a caller that holds the type as a value
(`BEEBUS_T_BOOL` … `BEEBUS_T_F64`) rather than as `T` — a gateway or a
configuration-driven node. A text topic (`BEEBUS_T_STRING`) passes the buffer as
`variable` and its size as `textCapacity`. `name` may be `nullptr`.

---

## Topics — moving data

```cpp
BeeBusResult publishAll(uint16_t topicID,
                        BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed);
BeeBusResult publishAll(const char* name,
                        BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed);
BeeBusResult publishTo(uint8_t nodeID, uint16_t topicID,
                       BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed);
BeeBusResult publishTo(uint8_t nodeID, const char* name,
                       BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed);
```

`publishAll()` sends the current value **once, to the whole ring**, and every
node holding that topic takes it — one section whatever the number listening,
which is what a node that wakes, reports and sleeps again wants: one turn on the
bus instead of one per listener.

Addressed to the ring rather than to a subscriber list because both transports
are shared: every node's receiver already has the bytes whatever the section
says, so a copy per listener buys no delivery and costs a copy.

```cpp
temperature = readSensor();
beebus.publishAll("temperature");   // one section; three subscribers read it
```

`publishTo()` sends it to one node, subscribed or not, on that node's own
stream. It is the only one that can be confirmed, because only a named node can
answer for itself: with `BeeBusDelivery::Confirmed` it completes on the
acknowledgement, the same path `writeTopic()` uses, so a command can be
published rather than written into a peer's topic.

A broadcast carries no sequence. A number belongs to one publisher counting to
one destination, which is what lets a subscriber on a deadband be sent one
sample in six without reading five as losses; a frame addressed to the ring has
no destination of its own, so the receiver neither measures it against a stream
nor records it into one. That is what keeps the two apart: a topic may be
broadcast **and** published to its subscribers on their own terms, and neither
disturbs the other. Nothing is given up, because a broadcast is unacknowledged
by definition and had no loss to detect.

You rarely need either call. A subscribed topic is published on the subscriber's
terms without anything from you. Use `publishAll()` for an **event** — a state
that changed and should go out this instant rather than waiting for the scan.

On a buffered topic, both record into the ring instead of sending.

Returns `Queued` when accepted, `NotMapped` if the topic is not bound,
`InvalidArgument` for a confirmed broadcast, `BufferFull` if a ring overwrote a
sample.

```cpp
BeeBusResult readTopic(uint8_t nodeID, uint16_t topicID);
BeeBusResult readTopic(uint8_t nodeID, const char* name);
```

Asks a peer for a value now. It lands in your registered variable; completion
arrives through `onReadComplete`, where `address` is the topic ID.

Returns `NotMapped` if the topic is not registered from that node — the reply
would have nowhere to go.

```cpp
template <typename T>
BeeBusResult writeTopic(uint8_t nodeID, uint16_t topicID, T value,
                        BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed,
                        bool wantResult = false);
template <typename T>
BeeBusResult writeTopic(uint8_t nodeID, const char* name, T value,
                        BeeBusDelivery delivery = BeeBusDelivery::Unconfirmed,
                        bool wantResult = false);
```

Writes into a peer's topic. The peer refuses it with `TypeMismatch` if it is not
the type they bound, or `Refused` if they made it read-only (or its range or
validate refused the value).

`Confirmed` (also spelt `BeeBusQoS::Reliable`) is acknowledged and retried, and
the owner applies it **once** however often it arrives. `wantResult` asks the
owner what it did with it — `ACCEPTED`, `CHANGED` or `REFUSED`, and the value it
kept — through `onTopicResult`; it makes the write confirmed. Writes to one topic
of one peer reach it in the order they were made.

```cpp
beebus.writeTopic(PUMP, "setpoint", 4.5f, BeeBusDelivery::Confirmed);
beebus.writeTopic(PUMP, 30, (int16_t)1, BeeBusQoS::Reliable, true);   // a command
```

```cpp
bool getTopic(uint16_t topicID, BeeBusScalar& value);
bool getTopic(const char* name, BeeBusScalar& value);
bool getRemoteTopic(uint8_t nodeID, uint16_t topicID, BeeBusScalar& value);
bool getRemoteTopic(uint8_t nodeID, const char* name, BeeBusScalar& value);
```

Reads a bound value generically. `getRemoteTopic` returns **false** until a value
has actually been received, which is a genuinely different answer from "the value
is zero".

Use `bb_scalarToFloat(value)` to get a number out whatever the type.

---

## Topics — subscriptions

```cpp
BeeBusResult subscribe(uint8_t nodeID, uint16_t topicID,
                       float deadband = 0.0f, bb_time_t maxIntervalMs = 0,
                       BeeBusQoS qos = BeeBusQoS::Live);
BeeBusResult subscribe(uint8_t nodeID, const char* name,
                       float deadband = 0.0f, bb_time_t maxIntervalMs = 0,
                       BeeBusQoS qos = BeeBusQoS::Live);

BeeBusResult unsubscribe(uint8_t nodeID, uint16_t topicID);
BeeBusResult unsubscribe(uint8_t nodeID, const char* name);
```

| | |
| --- | --- |
| `deadband` | In the value's own units. `0` reports every change. LIVE only. |
| `maxIntervalMs` | Report at least this often. `0` promises nothing. |
| `qos` | `Live`: best effort, newest wins. `Reliable`: every change, losses repaired with the current value ([05](05-topic-pdu.md#qos-live-and-reliable)). |

The terms belong to the subscription, so two subscribers may hold different ones
for the same topic.

Returns `NotMapped` if the topic is not registered from that node. `unsubscribe`
returns `NotSubscribed` if there was no subscription. A subscription that finds
the queue full, or the peer not yet in the ring, returns `Queued` and is sent
when it can be: subscribe everything at once. `QueueFull` comes back only when
the node has no room left to record another distinct pair of terms.

The interval is recorded as it will travel — rounded up to the nearest decisecond
or second — so quality is judged against what the publisher was really asked for.

```cpp
BeeBusResult subscribeAll(uint8_t nodeID, const char* pattern,
                          float deadband = 0.0f, bb_time_t maxIntervalMs = 0,
                          BeeBusQoS qos = BeeBusQoS::Live);
```

Subscribes to every topic of a peer whose name matches a glob: `*` for any run
of characters, `?` for exactly one.

**Requires a completed `fetchTopics` for that node**, and returns `NotMapped`
without one. Matches with no variable bound are passed over rather than half
subscribed. The expansion is paced over the following tokens.

---

## Topics — quality and freshness

```cpp
BeeBusQuality topicQuality(uint8_t nodeID, uint16_t topicID);
BeeBusQuality topicQuality(uint8_t nodeID, const char* name);

bb_time_t topicAge(uint8_t nodeID, uint16_t topicID);
bb_time_t topicAge(uint8_t nodeID, const char* name);
```

`topicQuality()` judges the value against the interval **your subscription asked
for**:

| | |
| --- | --- |
| `Unknown` | Never received, or not registered |
| `Good` | Within the interval, publisher in the ring |
| `Uncertain` | Late, but within twice the interval |
| `Bad` | Beyond that, or the publisher has left |

`topicAge()` is milliseconds since the value arrived, back-dated by the age a
buffered sample carried. `BEEBUS_SILENCE_NEVER` if none ever has.

```cpp
bool topicStream(uint8_t nodeID, uint16_t topicID, BeeBusTopicStream& out);

struct BeeBusTopicStream {
    uint16_t received = 0;   // Accepted publishes, wrapping
    uint16_t missed = 0;     // Publishes the sequence says never arrived, saturating
};
```

`topicStream()` is what a caller polls instead of registering for
`onTopicChanged` and `onTopicGap`. Both counters are running totals, so compare
them with what you saw last; two callers cannot take each other's edges.
Returns **false** when the topic is not registered here.

---

## Topics — store and forward

```cpp
bool     bufferTopic(uint16_t topicID, uint8_t depth);
bool     bufferTopic(const char* name, uint8_t depth);
uint16_t bufferedCount(uint16_t topicID);
uint16_t bufferedCount(const char* name);
```

Holds up to `depth` samples while there is nowhere to send them, oldest
overwritten by newest, and releases them in order with their true ages attached.

Depth `0` turns it off; the cap is **127** (`BEEBUS_TOPIC_BUFFER_MAX`). Returns
**false** if the topic is not bound locally or the depth is over the cap.

**Every publish of a buffered topic goes through the ring**, network up or down,
so a live value can never arrive ahead of the history behind it.

```cpp
beebus.bufferTopic("pressure", 20);
```

---

## Topics — commissioning

```cpp
BeeBusResult fetchTopics(uint8_t nodeID, bool withNames = true);
BeeBusResult publishTopicList(uint8_t nodeID, bool withNames = true);
BeeBusResult registerTopicWith(uint8_t nodeID, uint16_t topicID);
BeeBusResult registerTopicWith(uint8_t nodeID, const char* name);
```

`fetchTopics()` walks a peer's entire list, one page per token, chaining pages
automatically, reporting each entry through `onTopicList`.

`withNames = false` costs about 4 bytes an entry instead of 15–30. On a radio
link that is the difference between seconds and minutes.

`publishTopicList()` pushes ours the other way. `registerTopicWith()` announces
one topic.

```cpp
uint16_t topicCount();
bool     topicAt(uint16_t index, BeeBusTopicEntry& entry);
uint16_t discoveredCount(uint8_t nodeID);
bool     discoveredAt(uint8_t nodeID, uint16_t index, BeeBusTopicEntry& entry);
```

Enumerate what this node has bound, and what it has learnt about a peer.

```cpp
struct BeeBusTopicEntry {
    uint16_t topicID;
    uint8_t  type;      // bb_topicTypeName() for a readable form
    uint8_t  flags;     // BEEBUS_TOPIC_F_WRITABLE, BEEBUS_TOPIC_F_BUFFERED
    const char* name;   // up to 255 characters; "" when not asked for
};
```

`name` points at the node's own copy: a local topic's name while it is bound, a
discovered one's until the peer's list is fetched again, and in `onTopicList`
a received page's for the length of the call. Copy it to keep it.

---

## Topics — events

```cpp
void onTopicChanged(BeeBusTopicChangedHandler handler);
void onTopicWritten(BeeBusTopicWrittenHandler handler);
void onTopicStringChanged(BeeBusTopicStringChangedHandler handler);
void onTopicStringWritten(BeeBusTopicStringWrittenHandler handler);
void onTopicGap(BeeBusTopicGapHandler handler);
void onTopicComplete(BeeBusTopicCompleteHandler handler);
void onTopicList(BeeBusTopicListHandler handler);
void onTopicResult(BeeBusTopicResultHandler handler);
bool topicResult(uint8_t nodeID, uint16_t topicID, uint8_t& outcome,
                 BeeBusScalar& kept, uint16_t& count);
void onTopicDecided(BeeBusTopicDecidedHandler handler);

uint16_t topicWrites(uint16_t topicID);
bool     setTopicQueued(uint16_t topicID, bool queued);
void     topicApplied(uint16_t topicID, uint16_t settleMs = 0, bool command = false);
```

Every handler starts `(uint8_t nodeID, uint16_t topicID, ...)`, except
`onTopicList`, which receives `(uint8_t nodeID, const BeeBusTopicEntry& entry)`.
`onTopicResult` receives `(nodeID, topicID, uint8_t outcome,
const BeeBusScalar& kept, const char* keptText)` for a write made with
`wantResult`, once per write; `topicResult` is the same polled, for a caller
that takes no events: the last decision on a write of ours to that topic, and a
`count` that moves with each one (`BEEBUS_OUTCOME_NONE` when the write failed or
no decision came back within `rotationsResult` rotations). It returns **false**
when no such write was made.

`onTopicComplete` receives `(nodeID, topicID, uint8_t opcode, BeeBusResult
result)` for every topic operation this node issues — publish, read, write,
subscribe — with `opcode` the `BEEBUS_TOP_*` it answers, since more than one
operation may be outstanding on one topic. An unconfirmed operation completes
`Immediate` when it reaches the wire; a confirmed one when it is acknowledged,
or `Timeout`; a read when its reply arrives. **Set it before `connect()`**: the
handler is captured as each operation is accepted, so one installed later
misses everything already in flight.

`topicWrites()` is a running total of the writes peers have made to a topic
this node owns. `setTopicQueued(topicID, true)` makes a writable topic queue
its writes instead of storing them: a peer's write then reaches only
`onTopicWritten` / `onTopicStringWritten`, and the variable changes when the
program takes it. Returns **false** when the topic is not bound here.

`topicApplied` is for an owner whose topic is queued: call it once per write as
the program takes it, so a write that asked for a result is read back from
then, after the next `loop()` and `settleMs`; a `command` is answered
`ACCEPTED` at once. `onTopicDecided` tells the owner each decision it sent:
`(writer, topicID, outcome)`.

| | |
| --- | --- |
| `onTopicChanged` | A peer published a topic you registered. The variable is already updated. |
| `onTopicWritten` | A peer wrote a topic **you own**. Opposite direction; separate handler so the two are never confused. |
| `onTopicStringChanged` | The same as `onTopicChanged` for a text topic, carrying `(const char* text, uint16_t length)`. |
| `onTopicStringWritten` | The same as `onTopicWritten` for a text topic. |
| `onTopicGap` | Publishes went missing. The third argument is **how many**. |
| `onTopicComplete` | A topic operation you issued concluded. |
| `onTopicList` | One entry of a peer's list arrived. |
| `onTopicResult` | An owner decided a write of yours that asked. |
| `onTopicDecided` | You, as owner, decided a write that asked. |

**For a text topic use the string handler, not the scalar one.** An eight-byte
`BeeBusScalar` cannot carry text — and on a store-and-forward flush the bound
variable is written only for the newest sample while the handler fires for every
buffered entry, so reading the variable inside the handler would record the
newest message once per entry and lose the history.

```cpp
beebus.onTopicGap([](uint8_t node, uint16_t topicID, uint8_t missed) {
    logf("missed %u updates of topic %u from node %u", missed, topicID, node);
});
```

Handlers run inside `loop()`, while the frame that carried the value is being
processed, so they delay this node's token pass. **Keep them short.**

---

## Bidirectional binding

Layer 3, built from the calls above. One node **serves** a topic and decides
every write to it; any node **binds** it and keeps a variable equal to the
owner's. Change the bound variable and the change goes to the owner; the
owner's value comes back into the variable whatever it decided. Nothing to
pace, retry or time out.

```cpp
template <typename T>
bool serve(uint16_t topicID, T* variable, const BeeBusServe& terms = BeeBusServe());
template <typename T>
bool serve(const char* name, uint16_t topicID, T* variable, const BeeBusServe& terms = BeeBusServe());
bool serve(uint16_t topicID, char* text, uint16_t capacity, const BeeBusServe& terms = BeeBusServe());
bool unserve(uint16_t topicID);

struct BeeBusServe {
    bool writable = true;
    BeeBusServeKind kind = BeeBusServeKind::Value;   // or Command
    double min = 0, max = 0;          // refused outside, when min < max
    BeeBusValidateHandler validate;   // decides at once; see below
    uint16_t settleMs = 0;            // read-back delay after the next loop()
};
```

A write to a served topic is decided by, in turn: the range; the `validate`
callback, `uint8_t(uint8_t writer, uint16_t topicID, BeeBusScalar& value)`,
returning `BEEBUS_OUTCOME_ACCEPTED`, `BEEBUS_OUTCOME_CHANGED` (having changed
`value` to what it keeps) or `BEEBUS_OUTCOME_REFUSED`; and without one, a
**read-back** after the next `loop()` plus `settleMs` — the program runs in
between, so logic that puts a refused value back answers `CHANGED` with no extra
code. A `Command` serve answers once the write is taken, each write exactly once.

```cpp
template <typename T>
bool bind(uint8_t ownerID, uint16_t topicID, T* variable,
          const BeeBusBindTerms& terms = BeeBusBindTerms());
bool bind(uint8_t ownerID, uint16_t topicID, char* text, uint16_t capacity,
          const BeeBusBindTerms& terms = BeeBusBindTerms());
bool unbind(uint8_t ownerID, uint16_t topicID);

struct BeeBusBindTerms {
    BeeBusQoS qos = BeeBusQoS::Live;  // Reliable for setpoints, settings, configuration
    float deadband = 0.0f;            // LIVE only
};

BeeBusBindState bindState(uint8_t ownerID, uint16_t topicID);
bool bindResult(uint8_t ownerID, uint16_t topicID, uint8_t& outcome, BeeBusScalar& kept);
void bindChanged(uint8_t ownerID, uint16_t topicID);   // send now, not at the next loop()
void onBindState(BeeBusBindStateHandler handler);      // (owner, topicID, state), once per change
```

| State | Meaning |
| --- | --- |
| `Offline` | The owner is not in the ring |
| `Syncing` | Subscribed, the owner's value not yet here (also after it restarted) |
| `InSync` | The variable holds the owner's value |
| `Pending` | A change of yours is with the owner; its value waits for the decision |
| `Refused` | The owner kept another value, now in the variable (`bindResult` says which) |
| `Failed` | No decision came back within `rotationsResult` rotations (by default the owner's whole retry schedule for it: 64); the variable holds the owner's value |

`Refused` and `Failed` return to `InSync` on the next value from the owner.
A topic registered by hand cannot also be bound. A bind takes 80 bytes plus its
shadow copy of the value.

LIVE by default, like every primitive: a bind of a live value pays for no
receipts. A setpoint, a setting or configuration asks for RELIABLE, which no
lost frame can leave wrong. A change of the bound variable is a confirmed write
that asks for a result either way.

```cpp
BeeBusServe runTerms;
runTerms.min = 0;
runTerms.max = 1200;                                 // seconds; refused outside
plcBus.serve(20, &runTimeS, runTerms);               // the PLC

BeeBusBindTerms setting;
setting.qos = BeeBusQoS::Reliable;
panelBus.bind(PLC, 20, &shownRunTimeS, setting);     // the panel
shownRunTimeS = 1234;                                // -> Pending -> Refused, the PLC's value back
```

`serveRaw()` and `bindRaw()` take the type as a value, as the
[runtime-typed binding calls](#type-given-at-runtime) do:

```cpp
bool serveRaw(uint16_t topicID, uint8_t type, void* variable, const BeeBusServe& terms,
              const char* name, uint16_t textCapacity);
bool bindRaw(uint8_t ownerID, uint16_t topicID, uint8_t type, void* variable,
             const BeeBusBindTerms& terms, const char* name, uint16_t textCapacity);
```

---

## Registers — binding

```cpp
bool addCoil(uint16_t address, bool* variable);       // Peers read and write
bool addIsts(uint16_t address, bool* variable);       // Peers read only
bool addHreg(uint16_t address, int16_t* variable);    // Peers read and write
bool addIreg(uint16_t address, int16_t* variable);    // Peers read only

bool removeCoil(uint16_t address);
bool removeIsts(uint16_t address);
bool removeHreg(uint16_t address);
bool removeIreg(uint16_t address);
```

Returns **false** if the pointer is null, the address is already bound or covered
by a string, or the memory for the binding is not there. There is no other
limit on how many addresses a node binds.

```cpp
bool addRegister(BeeBusRegisterType table, uint16_t address, void* variable, uint8_t type);
bool removeRegister(BeeBusRegisterType table, uint16_t address);

uint16_t registerWrites(BeeBusRegisterType table, uint16_t address, uint16_t count = 1);
uint16_t registerWritesSince(BeeBusRegisterType table, uint16_t address, uint16_t count,
                             uint32_t& since, uint16_t* firstAddress = nullptr);
```

`addRegister` binds a value of any fixed-width type at its own width: a bit
table takes `BEEBUS_T_BOOL` only; a register table takes every fixed-width type,
`BOOL`, `I8`, `U8`, `I16` and `U16` over one register, `I32`, `U32` and `F32`
over two, and `I64`, `U64` and `F64` over four, **high word first**. It refuses
a type the table cannot hold, and a value any of whose registers is already
bound. A write of a number the type cannot hold is refused. See
[06 — Values wider or narrower than a register](06-modbus-pdu.md#values-wider-or-narrower-than-a-register).

`registerWrites` is a running total of a peer's writes across a range.
`registerWritesSince` says which values were written since the caller last
asked: keep a `uint32_t` per caller starting at 0; the call advances it, returns
how many values were written (a two-register value or a string counts once) and
sets `firstAddress` to the lowest.

### Lifetime

A binding stores the **address** of your variable, and the receive path reads and
writes through it for as long as the node runs. **The variable must outlive the
node, or be unbound before it is destroyed.**

```cpp
static int16_t level;            // Fine: outlives everything
beebus.addHreg(100, &level);

void configure() {
    int16_t local = 0;
    beebus.addHreg(101, &local); // Wrong: dangles the moment this returns
}

std::vector<int16_t> block(8);
beebus.addHreg(102, &block[0]);  // Wrong: a resize moves the storage
```

Nothing in the library can detect a stale binding — the pointer still looks
valid — so this is yours to get right. Static, global or member storage is the
usual answer. The same applies to `addString`, and to every topic binding.

---

## Registers — local access

```cpp
bool setCoil(uint16_t address, bool value);
bool setHreg(uint16_t address, int16_t value);
bool setIsts(uint16_t address, bool value);
bool setIreg(uint16_t address, int16_t value);

bool getCoil(uint16_t address, bool& value);
bool getHreg(uint16_t address, int16_t& value);
bool getIsts(uint16_t address, bool& value);
bool getIreg(uint16_t address, int16_t& value);
```

These go through the data map. You can equally just assign to your bound
variable — the map holds a pointer to it, so both are the same thing. A local
set raises **no** change event: that is your application writing to itself.

---

## Registers — remote access

### Single values

```cpp
BeeBusResult readHreg(uint8_t nodeID, uint16_t offset, int16_t& value, uint8_t fc = 0);
BeeBusResult readCoil(uint8_t nodeID, uint16_t offset, bool& value, uint8_t fc = 0);
BeeBusResult readIreg(uint8_t nodeID, uint16_t offset, int16_t& value, uint8_t fc = 0);
BeeBusResult readIsts(uint8_t nodeID, uint16_t offset, bool& value, uint8_t fc = 0);

BeeBusResult writeHreg(uint8_t nodeID, uint16_t offset, int16_t value, uint8_t fc = 0);
BeeBusResult writeCoil(uint8_t nodeID, uint16_t offset, bool value, uint8_t fc = 0);

BeeBusResult writeHreg(uint8_t nodeID, uint16_t offset, int16_t value, BeeBusDelivery delivery);
BeeBusResult writeCoil(uint8_t nodeID, uint16_t offset, bool value, BeeBusDelivery delivery);
```

Reads are asynchronous: the value appears in your variable when the answer
arrives, and `onReadComplete` tells you when.

**Reading your own node returns `Immediate`** and the value is already there.

`fc` overrides the function code. Leave it at 0 unless you have a specific reason.

### Arrays

```cpp
template<size_t N> BeeBusResult readHreg(uint8_t nodeID, uint16_t offset, int16_t (&values)[N], uint8_t fc = 0);
template<size_t N> BeeBusResult readCoil(uint8_t nodeID, uint16_t offset, bool (&values)[N], uint8_t fc = 0);
template<size_t N> BeeBusResult readIreg(uint8_t nodeID, uint16_t offset, int16_t (&values)[N], uint8_t fc = 0);
template<size_t N> BeeBusResult readIsts(uint8_t nodeID, uint16_t offset, bool (&values)[N], uint8_t fc = 0);

template<size_t N> BeeBusResult writeHreg(uint8_t nodeID, uint16_t offset, const int16_t (&values)[N], uint8_t fc = 0);
template<size_t N> BeeBusResult writeHreg(uint8_t nodeID, uint16_t offset, const int16_t (&values)[N], BeeBusDelivery delivery);
template<size_t N> BeeBusResult writeCoil(uint8_t nodeID, uint16_t offset, const bool (&values)[N], uint8_t fc = 0);
template<size_t N> BeeBusResult writeCoil(uint8_t nodeID, uint16_t offset, const bool (&values)[N], BeeBusDelivery delivery);
```

Fixed arrays size themselves from the type. For dynamic buffers:

```cpp
BeeBusResult readHregManual(uint8_t nodeID, uint16_t offset, int16_t* values, uint16_t numregs, uint8_t fc = 0, uint16_t stride = 0);
BeeBusResult readCoilManual(uint8_t nodeID, uint16_t offset, bool* values, uint16_t numcoils, uint8_t fc = 0, uint16_t stride = 0);
BeeBusResult readIregManual(uint8_t nodeID, uint16_t offset, int16_t* values, uint16_t numiregs, uint8_t fc = 0, uint16_t stride = 0);
BeeBusResult readIstsManual(uint8_t nodeID, uint16_t offset, bool* values, uint16_t numists, uint8_t fc = 0, uint16_t stride = 0);

BeeBusResult writeHregManual(uint8_t nodeID, uint16_t offset, const int16_t* values, uint16_t numregs, uint8_t fc = 0, uint16_t stride = 0);
BeeBusResult writeCoilManual(uint8_t nodeID, uint16_t offset, const bool* values, uint16_t numcoils, uint8_t fc = 0, uint16_t stride = 0);
BeeBusResult writeHregManual(uint8_t nodeID, uint16_t offset, const int16_t* values, uint16_t numregs, BeeBusDelivery delivery, uint16_t stride = 0);
BeeBusResult writeCoilManual(uint8_t nodeID, uint16_t offset, const bool* values, uint16_t numcoils, BeeBusDelivery delivery, uint16_t stride = 0);
```

`stride` is the gap in bytes between your elements. `0` means packed, which is
what a plain array is; give it a value when the elements are spaced more widely
than their own width — one field of an array of structs, for example.

Quantity limits are checked at the call: 125 registers or 2000 coils per read,
123 registers or 1968 coils per write. Over that is `InvalidArgument` immediately.

A write's payload is **copied when queued**, so the buffer you passed may go out
of scope.

---

## Strings

```cpp
bool addString(uint16_t address, char* buffer, uint16_t capacity);        // Holding registers
bool addInputString(uint16_t address, char* buffer, uint16_t capacity);   // Input registers
bool removeString(uint16_t address);
bool removeInputString(uint16_t address);

bool setString(uint16_t address, const char* text);
bool setInputString(uint16_t address, const char* text);
bool getString(uint16_t address, char* out, uint16_t size);
bool getInputString(uint16_t address, char* out, uint16_t size);

BeeBusResult readString(uint8_t nodeID, uint16_t address, char* buffer, uint16_t size);
BeeBusResult readInputString(uint8_t nodeID, uint16_t address, char* buffer, uint16_t size);
BeeBusResult writeString(uint8_t nodeID, uint16_t address, const char* text);
BeeBusResult writeString(uint8_t nodeID, uint16_t address, const char* text,
                         BeeBusDelivery delivery);
```

Two characters per register, big-endian, NUL terminated. A buffer of `capacity`
bytes carries `capacity - 1` characters and occupies `(capacity + 1) / 2`
registers; with an odd capacity the last register's second byte is past the
buffer, reads 0 and ignores writes.

`readString`'s `size` is **how many bytes to read**, not how much room you have.
It must fit your buffer and must not exceed the peer's field. Over 250 bytes is
refused at the call.

`writeString` costs the length of the message, not the capacity of the target
field. Over **245 characters** is refused — a split string could not be delivered
atomically.

Use `onReadComplete` for reads: an empty message and one that has not landed look
identical in the buffer.

---

## Failsafe

```cpp
void setCoilFailsafe(uint16_t address, BeeBusFailsafe action, bool value = false);
void setHregFailsafe(uint16_t address, BeeBusFailsafe action, int16_t value = 0);
bool setStringFailsafe(uint16_t address, BeeBusFailsafe action, const char* text = nullptr);

uint16_t failsafeCount(BeeBusRegisterType table, uint16_t address);

void onFailsafeApplied(BeeBusFailsafeHandler handler);
// void(BeeBusRegisterType type, uint16_t address, uint8_t lostNodeID)
```

Requires `BeeBusConfig::enableFailSafe = true`. Without a safe state stated, an
address holds whatever the lost node left it at — `HoldLastValue` is the default
everywhere. `setStringFailsafe` returns **false** if no string is bound at the
address. `failsafeCount` is how many times an address has been driven to its
safe state, as a running total.

```cpp
beebus.setCoilFailsafe(0, BeeBusFailsafe::ResetToValue, false);
```

> **A failsafe write does not raise the change handler.** `onCoilChanged` and
> `onHregChanged` report *a peer writing a value*; a failsafe is *this node
> acting because a peer stopped*. They are different events and only
> `onFailsafeApplied` reports the second.
>
> An application that centralises "an output changed, log it and update the
> display" on the change handler alone records every commanded change and misses
> every safety-driven one — the wrong way round for an incident review. If you
> care about the value moving, whatever moved it, register both:
>
> ```cpp
> beebus.onCoilChanged(0, [](uint16_t a, bool v, uint8_t from) { log(a, v, from); });
> beebus.onFailsafeApplied([](BeeBusRegisterType, uint16_t a, uint8_t lost) {
>     bool v = false;
>     beebus.getCoil(a, v);
>     log(a, v, lost);                    // Driven to safe, not commanded
> });
> ```

---

## Events and completion

```cpp
void onReadComplete(std::function<void(uint8_t nodeID, uint16_t address, BeeBusResult)>);
void onWriteComplete(std::function<void(uint8_t nodeID, uint16_t address, BeeBusResult)>);
```

`Immediate` means it succeeded. Anything else is a failure result — `Timeout`,
`NodeLost`, `Superseded`, `Refused`, `Cancelled`, `TypeMismatch`.

Both apply to operations issued **after** they are set. For topic operations the
`address` argument carries the topic ID.

```cpp
BeeBusRequestState requestState(uint8_t nodeID, uint16_t address, BeeBusResult& result);

enum class BeeBusRequestState : uint8_t { Idle, Pending, Done, Failed };
```

The same, polled, for a caller that registers no handler: where a read or write
to this peer and address has got to. `Pending` while it is outstanding, then
`Done` or `Failed` until the next request for the same address replaces it;
`result` says which failure.

```cpp
void onCoilChanged(uint16_t address, BeeBusCoilChangedHandler handler);
void onHregChanged(uint16_t address, BeeBusHregChangedHandler handler);
void onStringChanged(uint16_t address, BeeBusStringChangedHandler handler);
```

Raised when a **peer** writes a new value, once the whole request has been
applied, and only when the value actually changed. A string raises one event for
the message, not one per register.

```cpp
void onError(void (*handler)(BeeBusError error, const char* message));
```

Everything from CRC errors to membership changes to refused operations. Useful
during commissioning; a good place for a log line in production.

---

## Diagnostics

```cpp
bb_time_t getRotationTime();              // ms, 0 until measured
bb_time_t getHopTime(uint8_t nodeID);     // ms, 0 until measured
bb_time_t getHopTimeout(uint8_t nodeID);  // ms, 0 until measured
bb_time_t getSolicitWindow();             // ms
bb_time_t getSolicitWindowLimit();        // ms
uint16_t  getByteTime();                  // µs, 0 until measured
bb_time_t getServiceInterval();           // ms, 0 until measured
uint16_t  getPendingOpCount();
void      clearPendingOps();

BeeBusIOStats getLinkStatistics();
void          resetLinkStatistics();
```

`getRotationTime()` is the honest answer to "how quickly can this network
respond". `getHopTime()` is where a slow link on an otherwise fast ring shows up.

`getHopTimeout()` is what the same hop is allowed before the token is presumed
lost — the mean plus four times the spread measured on it. `getHopTime()` says
how *quick* a peer is, this says how *steady*, and the two together separate a
link that is merely slow from one that is going bad.

Every node times every hop it hears, not only its own, so these figures agree
across the ring rather than describing one node's corner of it. Walking
`getMemberAt()` and asking for each is a latency and health map of the whole
network.

`getSolicitWindow()` is the response window this node would state on its next
invitation to joining nodes, and `getSolicitWindowLimit()` the longest it will
hold the bus quiet for somebody else's. On a misbehaving segment, a window
climbing away from its settled figure is corruption being read as nodes
contending for a gap — on RS485 usually a pair with nothing driving it rather
than anybody waiting to join.

`getByteTime()` is what one byte costs on this bus, in microseconds, taken from
arriving frames; it is what raises the interframe gap on a slow link.
`getServiceInterval()` is how long this node takes between calls to `loop()`;
it floors every timeout the node derives. A figure here far above the others is
application code, not the network.

`getPendingOpCount()` climbing towards the queue depth (`maxPendingOps`, 50 on a
host and 16 on a board unless set) means you are queuing faster than the token
comes round.

```cpp
struct BeeBusIOStats {
    uint32_t framesReceived, framesSent;
    uint32_t crcErrors, framingErrors, bufferOverflows;
    uint32_t noiseBytesDiscarded, resyncs;
};
```

```cpp
void onDebug(void (*handler)(const char* category, const char* message));
void onFrameDebug(void (*handler)(const char* direction, const uint8_t* frame,
                                  uint16_t length, const char* decoded));
void onModbusDebug(void (*handler)(const char* type, uint8_t nodeID, const char* decoded));
```

The debug instance is **process-wide**, so these apply to every node in the
process; each message names the node it came from.

`onFrameDebug` costs a decode per frame — register it to diagnose a segment, not
in production. `direction` is `"TX"` or `"RX"`. `onModbusDebug` decodes every
Modbus PDU; `type` is `"REQUEST"` or `"RESPONSE"`.

Define `DISABLE_BEEBUS_DEBUG` to compile all of it out. Boards have it out by
default (define `BEEBUS_ENABLE_DEBUG` to keep it); the handlers then do nothing.

---

## Return values

```cpp
enum class BeeBusResult : uint8_t {
    Queued, Immediate,
    NotInitialised, InvalidArgument, UnknownNode, DuplicateNodeID, NotMapped,
    Duplicate, QueueFull, Timeout, NodeLost, Superseded, Cancelled, Refused,
    TypeMismatch, NotSubscribed, BufferFull,
};

inline bool bb_accepted(BeeBusResult r);      // Queued or Immediate
inline const char* bb_resultName(BeeBusResult r);
```

| Value | Meaning |
| --- | --- |
| `Queued` | Accepted; the result arrives on a later token |
| `Immediate` | Done now; the value is already in your variable |
| `NotInitialised` | `begin()` has not been called |
| `InvalidArgument` | Null pointer, zero count, or a quantity that cannot be expressed |
| `UnknownNode` | The destination is not currently on the network |
| `DuplicateNodeID` | This node's own ID is in use elsewhere, so it has stood down (`hasDuplicateNodeID()`) |
| `NotMapped` | Nothing is bound at that address or topic |
| `Duplicate` | An identical operation is already pending |
| `QueueFull` | The operation queue is at capacity |
| `Timeout` | Confirmed write: no acknowledgement after every retry |
| `NodeLost` | The destination left before it was delivered |
| `Superseded` | A newer write to the same address replaced it |
| `Cancelled` | Discarded by `clearPendingOps()` or a restart |
| `Refused` | The destination rejected it; the reason arrives via `onError` |
| `TypeMismatch` | The value's type differs from the binding at the other end |
| `NotSubscribed` | No subscription to act on |
| `BufferFull` | A store-and-forward ring overwrote its oldest sample |

The usual check:

```cpp
if (!bb_accepted(beebus.writeTopic(PUMP, "setpoint", 4.5f))) {
    // Handle it
}
```

---

## Enumerations

```cpp
enum class BeeBusDelivery : uint8_t { Unconfirmed, Confirmed,
                                      Live = Unconfirmed, Reliable = Confirmed };
typedef BeeBusDelivery BeeBusQoS;
enum class BeeBusServeKind : uint8_t { Value, Command };
enum class BeeBusBindState : uint8_t { Offline, Syncing, InSync, Pending, Refused, Failed };
enum class BeeBusFailsafe : uint8_t { HoldLastValue, ResetToValue };
enum class BeeBusQuality  : uint8_t { Unknown, Good, Uncertain, Bad };
enum class BeeBusRequestState : uint8_t { Idle, Pending, Done, Failed };

enum class BeeBusState : uint8_t {
    Offline, Listening, Joining, Requesting, Member, HoldingToken, PassingToken, Leaving
};

enum BeeBusRegisterType {
    MB_OUTPUT_COIL, MB_HOLDING_REGISTER, MB_INPUT_STATUS, MB_INPUT_REGISTER
};

enum class BeeBusError : uint8_t {
    CrcError, FrameError, BufferOverflow,
    MemberJoined, MemberLeft, NetworkTimeout, DuplicateNodeID,
    DuplicateToken,
    InvalidRequest, OperationError, OperationTimeout,
    ModbusError, InvalidFunction, InvalidAddress, DeviceFailure,
    StateChange, Unknown,
};
```

---

## Free functions

Names, for logging:

```cpp
const char* bb_resultName(BeeBusResult);
const char* bb_errorName(BeeBusError);
const char* bb_stateName(BeeBusState);
const char* bb_qualityName(BeeBusQuality);
const char* bb_topicTypeName(uint8_t type);
const char* bb_topicOpcodeName(uint8_t opcode);
const char* bb_topicReasonName(uint8_t reason);
const char* bb_outcomeName(uint8_t outcome);
const char* bb_bindStateName(BeeBusBindState state);
const char* bb_exceptionName(uint8_t code);
```

Values:

```cpp
struct BeeBusScalar {
    uint32_t bits = 0;               // The value, widened (low 32 bits of an 8-byte type)
    uint32_t high = 0;               // Upper 32 bits of an 8-byte type, else 0
    uint8_t  type = BEEBUS_T_BOOL;   // BEEBUS_T_*
};

float        bb_scalarToFloat(const BeeBusScalar& value);   // Any type, as a number
double       bb_scalarToDouble(const BeeBusScalar& value);  // Exact but for 64-bit integers past 2^53
uint64_t     bb_scalarBits64(const BeeBusScalar& value);    // high:bits, an 8-byte type's image
double       bb_bitsDouble(uint64_t bits);                  // An f64's image as a double
template <typename T> BeeBusScalar bb_scalarOf(T value);
bool         bb_scalarEqual(const BeeBusScalar& a, const BeeBusScalar& b);
```

Time:

```cpp
bb_time_t bb_millis();                                  // Wraps every ~49.7 days
bb_time_t bb_micros();                                  // Wraps every ~71.6 minutes
bb_time_t bb_elapsed(bb_time_t since, bb_time_t now);   // Correct across a wrap
bool      bb_isFuture(bb_time_t now, bb_time_t deadline);

void bb_setClockSource(bb_clock_fn millis, bb_clock_fn micros);
void bb_resetClockSource();
```

`bb_time_t` is fixed at **32 bits on every target**, so rollover behaves
identically everywhere. Always use `bb_elapsed()` rather than a bare subtraction:
it is correct across the wrap, and it says the wrap was considered.

Intervals:

```cpp
uint16_t  bb_encodeInterval(bb_time_t milliseconds);    // Rounds up
bb_time_t bb_decodeInterval(uint16_t coded);
```

---

## Constants worth knowing

| | |
| --- | --- |
| `BEEBUS_NODE_ID_MIN` / `MAX` | 1 / 254 |
| `BEEBUS_TOPIC_ID_MIN` / `MAX` | 1 / 32767 |
| `BEEBUS_TOPIC_NAME_MAX` | 256, including the terminator: 255 characters, the wire's length byte |
| `BEEBUS_TOPIC_NONE` | `0xFFFF`, "no such topic" |
| `BEEBUS_TOPIC_STRING_MAX` | 255, the longest text a topic carries |
| `BEEBUS_TOPIC_BUFFER_MAX` | 127 |
| `BEEBUS_OUTCOME_ACCEPTED` / `CHANGED` / `REFUSED` | 0 / 1 / 2, an owner's decision on a write; `BEEBUS_OUTCOME_NONE` (`0x7F`) when none came back |
| `BEEBUS_MAX_PENDING_OPS` | 50, the default queue depth on a host (16 on a board); `BeeBusConfig::maxPendingOps` sets any depth to 255 |
| `BEEBUS_MAX_PENDING_RESPONSES` | 50, likewise (`maxPendingResponses`) |
| `BEEBUS_MAX_RX_BUFFER` / `TX` | 512 |
| `BEEBUS_SILENCE_NEVER` | `0xFFFFFFFF` |
| `BEEBUS_AUTO` | 0, meaning "measure this" |
</content>
