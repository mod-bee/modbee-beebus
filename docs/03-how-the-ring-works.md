# 03 - How the ring works

This is the part of BeeBus you never call directly, and the part that decides
whether your network is any good. It is worth understanding even if you never
touch it.

## The shape of the thing

Every node keeps a list of the nodes it has heard taking part. It hands the
token to **the smallest member ID above its own, wrapping at the top**.

That is the entire ring construction. Nobody agrees on it, nobody is elected to
decide it, and nothing is negotiated. Two nodes with the same membership list
compute the same successor, so they compute the same ring.

Members 1, 4, 7, 20 give you:

```
   1  ──►  4  ──►  7  ──►  20
   ▲                        │
   └────────────────────────┘
```

Node 7 leaves. Every node notices, drops it from its list, and node 4 now
computes 20 as its successor. Nothing was renegotiated; the ring closed because
the rule produced a different answer from a shorter list.

A node alone on the bus is a ring of one. Its successor is itself. This is a
working state, not a failure — and it is also the bootstrap, which is why
starting a network needs no special case anywhere in the code.

## Holding the token

Only the node holding the token may transmit. When it has the token it:

1. Releases anything owed from a store-and-forward buffer.
2. Packs every queued operation and reply it can fit into one frame.
3. Names its successor in that frame's header, which *is* the token pass.
4. Waits for evidence the successor took it.

Step 2 is where the efficiency comes from. A node with a reading for node 4, a
setpoint for node 7 and a reply owed to node 20 sends **one frame** carrying all
three, each in its own length-prefixed section. See
[09 - Operations and framing](09-operations.md).

Every frame is both data and token. There is no separate token frame unless
there is genuinely nothing to say.

## Confirming the pass

The successor's own next transmission is the confirmation. There is no ack for
the token.

Any token frame heard from anywhere in the ring is accepted as confirmation,
not just one from the successor. That looks loose and is deliberate: if the
successor's first frame is lost, the next frame heard is from further round the
ring, and it still proves the token moved. Narrowing this to the successor alone
turns a single lost frame into a retry, and a retry into a duplicate token.

Only the successor's *own* frame is used to **time** the link, though —
measuring against somebody else's frame would be measuring whatever that node
was doing.

### The turn bit

The header carries an alternating `TURN` bit that flips each time a node takes
the token afresh, and stays the same across retransmissions. It answers a
question the successor cannot otherwise answer: *is this a new turn, or a second
copy of the one I already took?*

Acting on a duplicate would hand out a token that is already elsewhere in the
ring. The `REPEAT` bit is set alongside it, so a retransmission is also
distinguishable from a duplicate created by the medium.

### When a pass does not land

The holder retries, with a per-peer allowance and jitter. The jitter matters:
two nodes backing off on identical timers collide again on every attempt.

If retries run out, the ring does **not** immediately evict the node.

- **First failure:** go *round* it. The token is passed to the member after it
  instead. The node stays a member and is offered the token again next rotation.
  One rotation of failed passes is thin evidence on a link that loses frames,
  and evicting costs an announcement, a solicitation cycle and a rejoin.
- **Second failure in a row, and silent:** now it is removed, and the departure
  is announced in the header so everybody's list agrees.

A node that is still being heard from — even if it is refusing the token — is
never removed by the retry path. Silence removes nodes, not retries.

### If the token is lost entirely

Every member may regenerate it, but not at the same moment. Each waits its own
rank's worth of reclaim windows, so the lowest-numbered member goes first and
the next only if it does not. Restricting reclaim to one node would make that
node a single point of failure for the whole network.

A member only reclaims when the bus has actually gone *quiet*. A node retrying a
pass is transmitting the whole time, and regenerating underneath it would put a
second token on the bus.

If two tokens do end up circulating — say a frame was lost at exactly the wrong
moment — a node that hears somebody else passing the token while it holds one
drops its own. The other token is demonstrably circulating; ours is the
expendable one. IEEE 802.4 resolves it the same way.

## Joining: gap solicitation

Here is the elegant part.

A node passing the token to its successor is asserting something: *the address
space between me and my successor is empty*. Set the `SOLICIT` bit on that pass
and the assertion becomes an invitation:

> Anybody whose ID lies strictly between me and my successor may answer now.
> Successor: wait out the response window before using this token.

The gap needs no field of its own — it is already implied by the two node IDs in
the header. And because every member owns the gap between itself and its
successor, **the members' gaps tile the entire address space.** One rotation
invites every unused address in parallel.

A node alone on a silent bus is a ring of one whose successor is itself, so its
gap is *everything*. The bootstrap is the ordinary case.

The invitation carries two bytes: a **round number** and the **response
window**.

### Two nodes answer at once

They destroy each other's frames. The soliciting node sees a CRC error on an
otherwise quiet bus — nothing else can produce that during a window it opened
itself — and that is proof the gap is occupied.

The round number is what narrows the contention. A node answers a given round
**once**, and picks its moment inside the window from a hash of its own address
and that round number. Whoever speaks first wins the round; the rest stand down
until the next one, which re-rolls their positions because the round number
changed. So each round admits one node and the set still contending is strictly
smaller, however the addresses happen to fall.

A collision is the one case where backing off is exactly wrong, so it returns
the gap's owner to inviting on every rotation until a round finds nobody.

### Agreeing on the window

The window is **stated on the wire**, not worked out at each end. Both sides
must agree on it or an answer lands after the solicitor has stopped listening,
and two nodes measuring independently will not match: one that has just powered
up has timed nothing like as much as the ring it is joining. Stating it makes
the agreement structural and leaves the value free to be measured.

One byte carries it — a 3-bit exponent and 5-bit mantissa spanning 1 ms to about
4 seconds — so it costs the same on a wired segment holding a few milliseconds
and a radio holding hundreds.

### Announcing, and the sweep behind it

Normally a node does not wait to be invited. It sends a single byte into the gap
after a frame (see [04 - Frame format](04-frame-format.md)), and members that
hear it invite their own gap on the next turn.

That is what lets the periodic invitation slow down. An invitation costs nothing
to send — one bit in a header the frame carries anyway — but the whole ring
holds off for the window, and every member has a gap of its own to invite. On a
large ring, inviting on every rotation spends most of the rotation finding
nobody.

So the rate backs off toward a **sweep**, and the sweep exists only to catch an
announcement nobody heard. Crucially, it backs off **only once an announcement
has actually arrived at that node** — which is the proof that a joiner here can
be heard at all. Where one cannot land, a store-and-forward radio being the
case, nothing backs off and the sweep stays at every rotation. The medium
decides this, not a setting.

A gap that turns out to hold somebody returns to every rotation until a round
finds nobody, so a burst of arrivals is served at full speed.

### The joining sequence, end to end

```
Node powers up
   │
   ├─ Listening ──── heard traffic? ─── yes ──► Joining
   │                                 no  ──► start a ring of one
   │                                          (after a staggered wait)
   │
   ├─ Joining ────── solicited, and our ID is in the gap? ──► Requesting
   │                 nothing for a long time? ──► back to Listening
   │
   ├─ Requesting ─── wait for our contention slot, send a join request
   │                 ──► back to Joining
   │
   └─ Joining ────── handed the token ──► Member
```

The proof of a successful join is **being handed the token**. Until then the
node is a candidate, free to answer the next solicitation if its request was
lost or collided. A join request that is destroyed by a collision needs no
special handling: nobody heard it, so nothing changed, and the next solicitation
comes round.

The initial listen period is staggered per node ID, so a panel of nodes powered
up together do not all decide the bus is silent at the same instant and all
start rings.

## Failure and recovery

BeeBus has three separate ways of noticing something is wrong, because they are
three genuinely different faults.

**A peer went quiet.** Every node is handed the token once per rotation, so
being heard from is continuous evidence. A member not heard from for
16 rotations is removed. Nothing polls for this.

**The token went round without us.** A member is handed the token once per
rotation; hearing it go past repeatedly without arriving is proof the ring is
running without us. After roughly 8 passes per member the node drops its
membership and rejoins. This is checked in every ring state — including while
stalled on a pass — because a node evicted mid-pass would otherwise sit retrying
to a successor that will never confirm, and it cannot be solicited back in while
it still believes it is a member.

**We can transmit but not receive.** A node with a failed receiver would keep
transmitting for ever as its own ring of one, and unguarded those frames would
confirm token passes that never happened. So: no valid frame from anybody for
32 rotations, and the node withdraws and starts again from Listening.

That last one is also why almost everything in the receive path is guarded by
*is the sender a member of our ring*. A node shouting on the segment cannot
write itself into your membership with one frame.

### Two networks meeting

Two rings can end up on one bus — a cable is reconnected, a repeater comes back,
two halves of a plant are joined.

The rule: **the ring containing the lowest node ID absorbs the other.** Both
sides can evaluate that from what they can already see, both always reach the
same answer, and exactly one of them stands down and rejoins. Nothing has to be
said on the wire.

A node only concludes it is seeing a *different* ring when it recognises neither
end of a token pass. Knowing either end means this is plausibly our own ring
seen a moment out of date, and standing down on that would turn every lost frame
into a rejoin.

### Duplicate node IDs

A frame carrying our own address that we did not send means another node is
using it. This is checked before anything else is learnt from the frame, because
nothing downstream can make sense of a bus in that state.

The node that is already carrying the ring's traffic keeps the address; the
other stands down, and `DuplicateNodeID` is reported. Standing both down would
take out the working half of a commissioning mistake. The node that stood down
comes back by itself once the address has been quiet for long enough — usually
because the other device was taken off the network — or on the next
`connect()`, and `hasDuplicateNodeID()` says whether it is stood down now.

## Measured timing

**Every timeout in BeeBus is a number of token rotations, and a rotation is
measured, not configured.**

This is the design decision that makes one configuration suit a wired hop at
115200 baud and the same hop over a long-range radio.

### What is measured

| | |
| --- | --- |
| `T_rot` | A full rotation. Ring-wide. Drives membership, isolation, operation and response timeouts. |
| `T_hop[n]` | How long node *n* takes to accept the token. Per link, so a fast peer and a slow peer each get the timeout they deserve. |

**Every node times every hop, not only its own.** A pass to B followed by a
frame from B bounds that hop for anybody listening, and every frame reaches
every node, so each member keeps an estimate for each of the others rather than
for the one it happens to sit next to. The table is indexed by node ID and holds
the whole address space: an estimate evicted to make room is the one the ring is
about to need.

That table is the single source the rest of the timing is drawn from, and it is
open to callers — `getMemberAt()` walks the ring in token order, `getHopTime()`
gives a peer's mean and `getHopTimeout()` its allowance. Together they are a
latency and health map of the whole network, and a peer whose hop is
lengthening is a link going bad before it is a member gone.

Both are smoothed with TCP's round-trip estimator (RFC 6298) in integer
arithmetic: a mean, plus a variance term, with the timeout set at the mean plus
four times the observed spread. The variance term is what makes it fit a radio,
whose collision avoidance gives a modest mean and a wide spread.

Two rotation estimates are kept, for two different jobs:

- **Clean rotations** — any rotation that spanned a retry, a reclaim or a
  membership change is discarded (Karn's rule). This is the ring's true speed,
  and it is what `getRotationTime()` reports and what operation and retry timing
  is built from. It is also **computed** rather than measured end to end: the
  hops around the ring added up, each with its own spread. Hops have the
  response window taken off them, so a rotation summed from hops cannot contain
  a stall — which an interval measured from token to token would.
- **Observed rotations** — every rotation as it comes. This is what the ring is
  *managing right now*, and it drives the liveness windows, so a ring running
  slower widens its own failure-detection windows instead of tearing itself
  apart.

Because hops are measurable from outside the ring, a node that has never held
the token can still estimate a rotation before it joins.

### The rotation counts

Dimensionless on purpose — none of them changes when you add a node or a link
slows down.

| Constant | Rotations | What it bounds |
| --- | --- | --- |
| `BEEBUS_ROTATIONS_MEMBERSHIP` | 16 | Silence before a peer is gone |
| `BEEBUS_ROTATIONS_ISOLATION` | 32 | Silence before we conclude *we* are the problem |
| `BEEBUS_ROTATIONS_TOKEN_GIVEUP` | 2 | One wait for a successor to accept the token |
| `BEEBUS_ROTATIONS_STILL_THERE` | 4 | Silence before a peer refusing the token counts as gone |
| `BEEBUS_ROTATIONS_RECLAIM` | 4 | Dead air before regenerating the token, over and above the holder's retry budget |
| `BEEBUS_ROTATIONS_REJOIN` | 30 | Grace for a ring healing itself |
| `BEEBUS_ROTATIONS_OPERATION` | 16 | Round trip for a request |
| `BEEBUS_ROTATIONS_RETRY` | 1 | Backoff between attempts |
| `BEEBUS_ROTATIONS_FLUSH_SETTLE` | 4 | Settling time after joining before a buffer is emptied |
| `BEEBUS_PASSES_WITHOUT_TURN` | 8 (per member) | Passes heard with none for us before rejoining |

Because a node transmits at least once per rotation, surviving *k* rotations of
silence means surviving *k* consecutive losses. That is what these numbers
actually say.

Every derived timeout is clamped between a **floor** and **60 s at the top**.
The floor is what this node can be relied on to schedule within, so a fast
wired ring does not derive a timeout that ordinary loop jitter trips. It is
measured from how often the node actually services the bus — 50 ms stands in
until that is known, and it never goes below 5 ms — so a node running a tight
loop gets a tighter floor than one carrying heavy application code. The
ceiling stops one pathological sample from stretching failure detection out of
usefulness.

### Uneven links

A node whose hop costs at least 4× the average of the rest is given the token
every Nth rotation instead of every one, capped at every 4th.

This is what stops one 900 MHz radio hop from pacing an entire wired segment.
The ratio comes from the per-hop estimates, so a ring where every hop costs the
same skips nobody, and the cap keeps a skipped node's turn well inside its own
membership window.

A lap that skipped somebody is not counted as a rotation. Without that the
estimate would shrink, the slow hop would look like an even larger share, and
the node would be starved out — the feedback loop closes on itself.

### What you can see

```cpp
bb_time_t rotation = beebus.getRotationTime();       // ms, 0 until measured
bb_time_t hop      = beebus.getHopTime(PEER_ID);     // ms, 0 until measured
bb_time_t silence  = beebus.silenceFor(PEER_ID);     // ms since anything heard
bool      alive    = beebus.isAlive(PEER_ID);
```

`getRotationTime()` is the honest answer to "how quickly can this network
respond". `getHopTime()` is where a slow link on an otherwise fast ring shows
up — if one peer's hop time is ten times everybody else's, that is your radio,
or your wiring fault.

Both return 0 until the node has seen enough clean rotations to be sure of them
(four samples).

## Pinning a timing value

Every adaptive timing field in `BeeBusConfig` defaults to `BEEBUS_AUTO`. Setting
one to a number pins that field and switches adaptation off **for it alone**; the rest of
the node keeps tuning itself.

```cpp
BeeBusConfig config;
config.nodeTimeout = 30000;         // Pin one thing
beebus.begin(&transport, 1, config);
```

A pinned field states one member of its family, and the others keep their
proportion to it. Pinning `nodeTimeout` does not collapse "is this peer gone"
and "has this peer refused the token twice" onto the same test.

**You should almost never need this.** The measured values are better than a
guess, they follow the network as it changes, and they do not need revisiting
when you add a node. See [10 - Configuration](10-configuration.md).

## Next

- [04 - Frame format](04-frame-format.md) — the bytes that carry all of the above.
- [09 - Operations and framing](09-operations.md) — what rides along with the token.
</content>
