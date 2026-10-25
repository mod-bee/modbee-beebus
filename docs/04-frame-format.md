# 04 - Frame format

Every byte on the wire, with worked examples. If you are implementing BeeBus
somewhere else, this chapter and the two PDU chapters are the specification.

## The shape of a frame

```
FLAG │ SRC │ NEXT │ FLAGS │ [optional fields] │ [sections] │ CRC16 │ FLAG
0x7E │  1  │  1   │   1   │      0..3 bytes   │  0..n      │   2   │ 0x7E
```

Only `SRC`, `NEXT`, `FLAGS` and the CRC are always present. Everything else is
announced by a bit in `FLAGS`. The shortest legal frame is **6 bytes**
(`BEEBUS_MIN_FRAME_LEN`), and the longest is **512** (`BEEBUS_MAX_TX_BUFFER`).

All multi-byte integers are **big-endian**, Modbus and BeeBus alike.

## Delimiting and escaping

BeeBus uses HDLC-style framing. `0x7E` is the frame delimiter and appears at
both ends of every frame.

Inside the frame body, two bytes are escaped:

| Byte | Becomes |
| --- | --- |
| `0x7E` (FLAG) | `0x7D 0x5E` |
| `0x7D` (ESC) | `0x7D 0x5D` |

The escape is `0x7D` followed by the original byte XORed with `0x20`.

Because `0x7E` can therefore never occur inside a frame, it unambiguously ends
the frame in progress and starts the next one. A receiver that loses
synchronisation — noise, a partial frame, a wrong baud rate — always recovers on
the next `0x7E`.

**The opening FLAG is not escaped and is part of the CRC.** The frame marker at
buffer position 0 is covered by the CRC calculation and becomes the opening flag
on the wire, written raw. A frame that lost its opening flag would therefore
verify against the wrong bytes and be rejected, which is the intent.

Anything arriving outside a frame is discarded immediately and counted in
`noiseBytesDiscarded`. This is what stops a floating bus — reading as sustained
`0xFF` or `0x00` — from filling the receive buffer.

## The CRC

Standard Modbus CRC-16: reflected polynomial `0xA001`, initial value `0xFFFF`.

It is computed over the **unescaped** frame body starting at the opening FLAG
and ending just before the CRC itself, then appended **big-endian**. (Note that
this is the opposite byte order from Modbus RTU, which sends its CRC
little-endian.)

```cpp
uint16_t crc = 0xFFFF;
for (each byte b) {
    crc ^= b;
    for (8 times) crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
}
```

Escaping is applied *after* the CRC is computed, and removed *before* it is
checked. The CRC protects the logical frame, not its wire encoding.

## The header

### SRC and NEXT

| | |
| --- | --- |
| `SRC` | The node that sent this frame. 1–254. |
| `NEXT` | The node the token is being handed to, or **0** when the frame carries no token. |

`NEXT == SRC` means a node handing the token to itself: a ring of one. Other
nodes treat that as *not taking part in our ring*, which is what stops a node
with a failed receiver from confirming token passes that never happened.

### FLAGS

| Bit | Name | Pulls in |
| --- | --- | --- |
| `0x01` | `SOLICIT` | 2 bytes: the round number, then the response window |
| `0x02` | `MEMBER_JOINED` | 1 byte: the node that joined |
| `0x04` | `MEMBER_LEFT` | 1 byte: the node that left |
| `0x08` | `HAS_SECTIONS` | nothing; sections follow the header |
| `0x10` | `TURN` | nothing; the alternating turn bit |
| `0x20` | `REPEAT` | nothing; this frame repeats one already sent |

The optional value bytes follow the header **in bit order**, so a receiver can
walk them knowing nothing but the flags byte.

```
FLAGS = 0x07  →  FLAG SRC NEXT 0x07 [round] [window] [joined] [left] ... CRC FLAG
FLAGS = 0x04  →  FLAG SRC NEXT 0x04 [left] ... CRC FLAG
```

`0x40` and `0x80` are unassigned.

**What each one means:**

- **`SOLICIT`** — "anybody whose ID lies strictly between `SRC` and `NEXT` may
  answer now; successor, wait out the response window first."

  The **round number** is what narrows the contention: a node answers a given
  round once, and whoever speaks first in it wins, so each round admits one node
  and the set still contending is strictly smaller.

  The **window** is the response window the solicitor is holding, encoded
  logarithmically into one byte: a 3-bit exponent and a 5-bit mantissa spanning
  1 ms to about 4 seconds, rounded up. It is stated rather than derived because
  both sides must agree on it, and a figure each works out from its own
  measurements will not match — a node that has just powered up has timed
  nothing like as much as the ring it is joining.

  See [03 - How the ring works](03-how-the-ring-works.md).
- **`MEMBER_JOINED`** — two meanings, distinguished by whether the named node is
  the sender. *First person* (`joined == SRC`) is a node asking to join, having
  been solicited. *Third person* is a member announcing somebody else's arrival.
- **`MEMBER_LEFT`** — likewise. *First person* is a node announcing its own
  departure; *third person* is a member reporting one it removed.
- **`TURN`** — flips each time a node takes the token afresh, held across
  retransmissions, so a successor can tell a new turn from a second copy of one
  it already took.
- **`REPEAT`** — the sender has transmitted this frame before. Separates a
  retransmission from a duplicate created by the medium.

Departures are processed before arrivals, so a frame that removes one node and
admits another leaves membership in exactly the state the sender described.

## Sections

When `HAS_SECTIONS` is set, one or more length-prefixed sections follow the
header, back to back, up to the CRC.

```
┌────────────────┬──────┬───────────────┐
│  HEADER (16)   │ DEST │  PDU ...      │
└────────────────┴──────┴───────────────┘
       2 bytes      1        LENGTH-1
```

The 16-bit section header is **not** just a length. Its bits are:

```
 15    14   13 12    11 10  9     8 .. 0
┌─────┬─────┬────────┬──────────┬───────────────────┐
│ ACK │ RSP │ resvd  │ PDU TYPE │  LENGTH (9 bits)  │
└─────┴─────┴────────┴──────────┴───────────────────┘
 0x8000 0x4000 0x3000    0x0E00        0x01FF
```

| Field | Mask | Meaning |
| --- | --- | --- |
| `LENGTH` | `0x01FF` | Bytes in `DEST` + `PDU`. 2–511. |
| `PDU TYPE` | `0x0E00` | What kind of payload this is. Shift right 9. |
| reserved | `0x3000` | Must be zero. Masked off on read. |
| `RSP` | `0x4000` | This section answers a request rather than making one. |
| `ACK` | `0x8000` | The last two bytes of the section are the sender's operation ID, not PDU content. |

**Why `RSP` exists.** A Modbus read request and a read response share their
function code, and at one length they share their shape as well: a request is
`FC + address(2) + quantity(2)`, five bytes, and a response is
`FC + address(2) + byteCount + data`, which is also five bytes when the byte
count is one. The quantity's high byte sits exactly where the byte count does,
so a receiver guessing from structure reads a request for 256–511 coils as a
response and silently never executes it. Stating the direction removes the
guess.

A receiver that does not know the bit ignores it and falls back to guessing, and
a sender that does not set it leaves the receiver guessing, so a ring may hold
both versions while it is being upgraded. The fallback is not ambiguous
either: a register read cannot answer one byte, and for a bit read the receiver
asks whether it has such a read outstanding to that peer, since only a node that
asked can be answered.

`DEST` is a node ID, 1 to 254, and a receiver walks past any section not
addressed to it. **255 is a broadcast**: every node reads the section, and
whoever holds the topic it names takes the value. Only topic sections carry it,
because nothing acknowledges a broadcast — which suits a value being reported
and not a register being written. It is what `publishAll()` sends, so a node
reporting to several subscribers spends one section rather than one each.

**The length must be masked before it is used for anything.** Treating a flag
bit as part of the length pushes the section end past the frame and silently
discards this section and every one after it.

### PDU types

| Value | Family |
| --- | --- |
| `0` | Modbus — see [06 - The Modbus PDU](06-modbus-pdu.md) |
| `1` | Topic — see [05 - The topic PDU](05-topic-pdu.md) |

Modbus is 0 deliberately: a section with the type bits clear is a plain Modbus
section, so the register model needs nothing extra on the wire.

**A receiver that meets a PDU type it does not know skips that section and keeps
walking.** Because sections are length-prefixed, skipping costs nothing and the
rest of the frame is still processed. A node meeting a newer peer loses only
what it could not have understood anyway. This is the extension mechanism.

### DEST

Each section is addressed independently. `DEST` is the node this section is for;
a node that is not the destination ignores the section and moves to the next.

One frame can therefore carry a reading for node 4, a setpoint for node 7 and a
reply owed to node 20, and all three arrive in the same token turn. This is
where BeeBus's efficiency on slow links comes from.

### Acknowledgement

When the `ACK` bit is set, the last **2 bytes** of the section are the sender's
operation ID (big-endian), and they are *not* part of the PDU. They must be
removed before the PDU is parsed.

The receiver echoes that ID back in an acknowledgement section:

```
FC 0x46 │ opId(2) │ status │ opId(2) │ status │ ...
```

The entry count is implied by the section length — 3 bytes each, up to 32
entries. `status` is `0x00` for success, or a Modbus exception code explaining
the refusal. Several confirmed writes from one node in one token turn are
batched into a single acknowledgement section, costing 3 bytes each rather than
a section apiece.

`0x46` comes from the Modbus user-defined range `0x41`–`0x48`, so it cannot
collide with a standard function code, and it is deliberately not a write code —
the section classifier treats write codes as requests, and would re-execute the
write on the requester.

## Worked examples

Every CRC below is real. "body" is what the CRC is computed over; "wire" is what
goes out, escaped and flagged.

### A bare token pass, node 1 to node 4

```
body:  7E 01 04 10                                CRC: C0 4A
wire:  7E 01 04 10 C0 4A 7E
       ▲  ▲  ▲  ▲
       │  │  │  └─ FLAGS: TURN
       │  │  └──── NEXT = 4
       │  └─────── SRC = 1
       └────────── FLAG
```

Six bytes. This is what an idle ring looks like.

### A token pass carrying a solicitation

```
body:  7E 01 04 11 01 14                          CRC: 6F 67
wire:  7E 01 04 11 01 14 6F 67 7E
                ▲  ▲  ▲
                │  │  └─ response window 0x14 = 20 ms
                │  └──── round 1
                └─────── FLAGS: TURN | SOLICIT
```

"Node 1 to node 4; anybody with an ID of 2 or 3 may answer now, within the
next 20 ms."

### A join request from node 3

```
body:  7E 03 00 02 03                             CRC: 2F 4D
wire:  7E 03 00 02 03 2F 4D 7E
          ▲  ▲  ▲  ▲
          │  │  │  └─ joined = 3 (the sender itself: first person)
          │  │  └──── FLAGS: MEMBER_JOINED
          │  └─────── NEXT = 0, no token
          └────────── SRC = 3
```

### Node 1 removing node 7 and passing to node 20

```
body:  7E 01 14 14 07                             CRC: 30 03
wire:  7E 01 14 14 07 30 03 7E
          ▲  ▲  ▲  ▲
          │  │  │  └─ left = 7
          │  │  └──── FLAGS: TURN | MEMBER_LEFT
          │  └─────── NEXT = 20
          └────────── SRC = 1
```

### A topic publish riding on the token

Node 1 passes the token to node 4, and carries a `PUBLISH` of topic 1 —
sequence 7, type `f32`, value 21.5 — addressed to node 2.

```
body:  7E 01 04 18 02 09 02 00 01 07 07 41 AC 00 00      CRC: A7 22
wire:  7E 01 04 18 02 09 02 00 01 07 07 41 AC 00 00 A7 22 7E
                ▲  ─────  ▲  ──────────────────────
                │    │    │            │
                │    │    │            └─ PDU: PUBLISH, topic 1, seq 7, f32, 21.5
                │    │    └─────────────── DEST = 2
                │    └──────────────────── section header 0x0209
                │                            = PDU type 1 (topic), length 9
                └───────────────────────── FLAGS: TURN | HAS_SECTIONS
```

Section length 9 = 1 destination byte + 8 PDU bytes.

### A confirmed topic write

Node 2 passes the token to node 7, and writes topic 10 on node 1 to the `int16`
value 750, asking to be acknowledged. Operation ID 42; the write's request ID
(protocol version 2, applied once at the owner) is 1, node 2's session `0x1234`,
and node 1's session as node 2 knows it `0xB98F`.

```
body:  7E 02 07 18 82 0F 01 01 0A 01 00 01 12 34 B9 8F 03 02 EE 00 2A      CRC: 63 70
wire:  7E 02 07 18 82 0F 01 01 0A 01 00 01 12 34 B9 8F 03 02 EE 00 2A 63 70 7E
                   ─────  ▲  ────────────────────────────────── ─────
                     │    │                   │                    │
                     │    │                   │                    └─ operation ID 42
                     │    │                   └────────────────────── PDU: WRITE, topic 10,
                     │    │                                             flags REQUEST, request 1,
                     │    │                                             sessions 0x1234 / 0xB98F,
                     │    │                                             i16, 750
                     │    └──────────────────────────────────────── DEST = 1
                     └───────────────────────────────────────────── section header 0x820F
                                                                      = ACK | type 1 | length 15
```

Section length 15 = 1 destination + 12 PDU + 2 operation ID.

Node 1 will answer, on its own next token, with an acknowledgement section
containing `00 2A 00` — operation 42, status OK. A retry of the same operation
carries the same request ID, and is acknowledged without being applied again;
a node 1 that has restarted since (another session) refuses it instead.

### Numbering as a frame is built

Two numbers are given to a topic section only as it goes into a frame, in frame
order, never when it is queued: the RELIABLE stream number of a value (a
`PUBLISH` or `SUBACK` with type bit `0x40`) and the request ID of a confirmed
write. Answers leave after operations in a frame, so a number given at queue
time would arrive out of order; and a number is never spent on something that
did not go. A `RESULT`'s value is refreshed the same way. See
[05 - The topic PDU](05-topic-pdu.md#qos-live-and-reliable).

## Receive-path limits

| | |
| --- | --- |
| `BEEBUS_MAX_RX_BUFFER` / `BEEBUS_MAX_TX_BUFFER` | 512 bytes |
| Frame assembly queue | 5 complete frames |
| `BEEBUS_MAX_RX_BYTES_PER_LOOP` | 1024 bytes drained per `loop()` |

The per-loop bound is what keeps a flooded node responsive: leftover bytes stay
in the transport and are taken next call, so the application loop still runs.

An assembled frame longer than the buffer is abandoned and the framer
resynchronises, counted in `resyncs`.

## Link statistics

```cpp
BeeBusIOStats stats = beebus.getLinkStatistics();
```

| Field | What it tells you |
| --- | --- |
| `framesReceived`, `framesSent` | Traffic volume |
| `crcErrors` | Corrupt frames. **The number to judge a link by.** |
| `framingErrors` | Runt frames between two flags |
| `bufferOverflows` | The frame queue filled — the application is not calling `loop()` often enough |
| `noiseBytesDiscarded` | Bytes seen outside any frame |
| `resyncs` | Oversized partial frames abandoned |

A rising `crcErrors` on an otherwise quiet bus means collisions, reflections or
a marginal link. A steadily growing `noiseBytesDiscarded` means a floating bus,
a missing termination resistor, or a baud rate mismatch — a quite different
fault, and worth distinguishing.

Note that `crcErrors` also rises legitimately during joining: two nodes
answering one solicitation destroy each other's frames, and that collision is
exactly how BeeBus discovers the gap is occupied.

## The announcement byte

One byte carries no frame and needs none. A node waiting to be invited sends
`0x55` alone into the gap that follows a frame, and members that hear it invite
their own gap on the next turn instead of waiting for the periodic sweep.

It is read as a body one byte long, delimited by the closing flag of the frame
it follows and the opening flag of the next — too short to be a frame, and so
unambiguous where it sits. A frame would not fit: the interframe gap guarantees
room for two byte times and a control frame needs a dozen, so a joiner that sent
one would be transmitted over before it finished.

It admits nobody and carries no address. All it says is that somebody is
waiting, which is why it needs neither. A byte the line invents costs one
invitation that finds nothing.

On a store-and-forward radio it does not arrive: the module buffers bytes and
hands them over whenever it packetises, long after the gap they were aimed at.
Nothing special is done about that. Members only slow their invitations down
once an announcement has actually been heard, so on such a link they never do,
and the sweep stays at every rotation.
</content>
