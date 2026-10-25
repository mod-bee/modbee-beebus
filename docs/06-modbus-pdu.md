# 06 - The Modbus PDU

The compatibility model. Coils, discrete inputs, holding registers and input
registers, addressed and behaving as Modbus, so that something outside BeeBus
can look in.

Sections carrying these PDUs have **PDU type 0** in the section header.

## Why this exists

Topics are better in every way that matters to your own nodes: typed, named,
self-describing, with quality and gap detection.

None of that helps when the thing that needs the data is a SCADA package, a PLC,
an HMI, or a Modbus TCP gateway. Those speak registers. So BeeBus speaks
registers too, with the same address semantics and the same function codes, and
a stock Modbus tool pointed at a gateway sees an ordinary register map.

**You can have both.** The same C++ variable can be bound as a topic *and* as
registers:

```cpp
float pressure = 0.0f;

beebus.addInputTopic("pressure", 1, &pressure);                      // Your nodes see a float
beebus.addRegister(MB_INPUT_REGISTER, 30, &pressure, BEEBUS_T_F32);  // The SCADA sees registers 30..31
```

No translation layer, no synchronisation code — two views of your data.

## The four register spaces

Exactly as Modbus defines them:

| Space | Type | Peers may | Bind with |
| --- | --- | --- | --- |
| Coils | `bool` | read and write | `addCoil()` |
| Discrete inputs | `bool` | read only | `addIsts()` |
| Holding registers | `int16_t` | read and write | `addHreg()` |
| Input registers | `int16_t` | read only | `addIreg()` |

Addresses are 16-bit, 0–65535, and each space has its own address range. Coil 100
and holding register 100 are different things.

**Binding is per address.** A node can bind every address of all four spaces;
the map takes memory as it grows, and a bind is refused only when that memory
is not there.

```cpp
bool    pumpRun    = false;
bool    levelHigh  = false;
int16_t setpoint   = 0;
int16_t levelRaw   = 0;

beebus.addCoil(0,  &pumpRun);      // Peers may switch it
beebus.addIsts(0,  &levelHigh);    // Peers may only read it
beebus.addHreg(40, &setpoint);     // Peers may set it
beebus.addIreg(30, &levelRaw);     // Peers may only read it
```

An address with nothing bound to it does not exist, and a read or write to it is
refused with `illegal data address`. There is no shadow register file.

## Function codes

| Code | Operation | Limit per PDU |
| --- | --- | --- |
| `0x01` | Read coils | 2000 |
| `0x02` | Read discrete inputs | 2000 |
| `0x03` | Read holding registers | 125 |
| `0x04` | Read input registers | 125 |
| `0x05` | Write single coil | 1 |
| `0x06` | Write single register | 1 |
| `0x0F` | Write multiple coils | 1968 |
| `0x10` | Write multiple registers | 123 |
| `0x46` | Acknowledgement (BeeBus) | 32 entries |

The quantity limits are what fits in one PDU, and they are checked **at the
call**, not after a round trip. `readHreg(node, 0, buf)` with a 200-element
buffer returns `InvalidArgument` immediately rather than being queued,
transmitted, and answered with an exception a rotation later.

`0x46` is from the Modbus user-defined range `0x41`–`0x48`, so it can never
collide with a standard code.

## Wire layouts

Requests are standard Modbus. **Responses differ in one deliberate way**, noted
below.

### Read request — 0x01, 0x02, 0x03, 0x04

```
┌──────┬───────────────┬────────────┐
│  FC  │ start address │  quantity  │
│  1   │       2       │      2     │
└──────┴───────────────┴────────────┘
```

### Read response

```
┌──────┬───────────────┬────────────┬──────────┐
│  FC  │ start address │ byte count │  data    │
│  1   │       2       │      1     │  n       │
└──────┴───────────────┴────────────┴──────────┘
```

> **This is the one place BeeBus departs from stock Modbus.** A standard Modbus
> read response carries no address, because Modbus is strictly one request at a
> time on a line owned by a master.
>
> BeeBus pipelines. A node can have several reads outstanding to one peer in one
> token turn, and their answers can come back in the same frame. Without the
> address there would be no way to attribute a response to the request that
> caused it. The three extra bytes buy correctness.

Register data is two bytes per register, big-endian. Bit data is packed **least
significant bit first** — coil *n* of the request is bit `n % 8` of byte
`n / 8`.

A response is marked as one by the `RSP` bit in its section header (see
[04 - Frame format](04-frame-format.md)), because the layout above and the read
request layout coincide at five bytes and a receiver cannot always tell them
apart by shape.

### Write single — 0x05, 0x06

```
┌──────┬───────────────┬────────────┐
│  FC  │    address    │   value    │
│  1   │       2       │      2     │
└──────┴───────────────┴────────────┘
```

Coil values are `0xFF00` for true and `0x0000` for false, as Modbus specifies.

### Write multiple — 0x0F, 0x10

```
┌──────┬───────────────┬────────────┬────────────┬──────────┐
│  FC  │ start address │  quantity  │ byte count │  data    │
│  1   │       2       │      2     │      1     │  n       │
└──────┴───────────────┴────────────┴────────────┴──────────┘
```

The byte count and the quantity are two statements about the same payload; when
they disagree the request is refused, because unpacking as far as the quantity
claims would read past what the sender supplied.

**A block write is all or nothing.** If any address in the range refuses the
write, everything already applied is put back and the whole request is answered
with `SLAVE DEVICE FAILURE`. A refusal therefore means *nothing changed*, which
is what an integrator has to be able to rely on: a half-applied setpoint block —
a PID triple, a recipe, a ramp profile — leaves the plant running a combination
nobody commanded while the HMI reports the write failed.

### Write responses

**Writes are not echoed.** Stock Modbus echoes a write request back as its
response; BeeBus does not, because the ring already orders delivery between a
pair of nodes and the echo would be pure airtime.

If the sender asked to be told the outcome (`BeeBusDelivery::Confirmed`), it
gets an acknowledgement section instead, carrying the operation ID rather than
repeating the payload:

```
┌──────┬────────────┬────────┬────────────┬────────┬─────
│ 0x46 │ opId       │ status │ opId       │ status │ ...
│  1   │     2      │   1    │     2      │   1    │
└──────┴────────────┴────────┴────────────┴────────┴─────
```

The entry count is implied by the section length. `status` is `0x00` for
success, or the Modbus exception code that refused the write — **a refusal
travels back just as a success does**, because the sender is waiting on that ID
and will keep retrying until it hears something.

Up to 32 acknowledgements to one node share a single section, so several
confirmed writes in one token turn cost 3 bytes each rather than a section apiece.

### Exception response

```
┌───────────┬───────────────┬──────────────────┐
│ FC | 0x80 │ start address │  exception code  │
│     1     │       2       │        1         │
└───────────┴───────────────┴──────────────────┘
```

The address is carried for the same reason read responses carry one: an
exception with no address cannot be attributed to the request that caused it.

| Code | Meaning | `onError` (`BeeBusError`) | Operation result (`BeeBusResult`) |
| --- | --- | --- | --- |
| `0x01` | Illegal function | `InvalidFunction` | `Refused` |
| `0x02` | Illegal data address | `InvalidAddress` | `NotMapped` |
| `0x03` | Illegal data value | `InvalidRequest` | `InvalidArgument` |
| `0x04` | Slave device failure | `DeviceFailure` | `Refused` |

An exception carries no data, so the caller's variable is **left alone**.
Overwriting it would present a fabricated reading as a real one. The operation
is dropped rather than retried, because the faults reported here are decisions
the responder has already made and will make again.

## Reading and writing

### Single values

```cpp
int16_t value = 0;
beebus.readHreg(NODE, 100, value);            // Lands in `value` on a later token
beebus.writeHreg(NODE, 100, 750);             // Unconfirmed
beebus.writeHreg(NODE, 100, 750, BeeBusDelivery::Confirmed);

bool state = false;
beebus.readCoil(NODE, 5, state);
beebus.writeCoil(NODE, 5, true, BeeBusDelivery::Confirmed);
```

A read is **asynchronous**. It returns `Queued`, and the value appears in your
variable when the answer arrives.

Reading your *own* node is different — it returns `Immediate` and the value is
already there, because there is nothing to send.

### Arrays

Fixed-size arrays size themselves:

```cpp
int16_t values[10];
beebus.readHreg(NODE, 100, values);           // Reads 10, no count needed

bool states[16];
beebus.writeCoil(NODE, 0, states);            // Writes 16
```

Dynamic ones state the count:

```cpp
beebus.readHregManual(NODE, 100, ptr, count);
beebus.writeHregManual(NODE, 100, ptr, count, BeeBusDelivery::Confirmed);
```

**One block read of 20 registers is one PDU. Twenty single reads are twenty.**
On a slow link this is the single biggest thing you control.

### Values wider or narrower than a register

`addHreg` binds an `int16_t`. Anything else — a `float` setpoint, an `int32_t`
counter, a `uint8_t` node number, a `bool` among numbers — binds at its own
width with `addRegister`, from the address given:

```cpp
float   levelHi  = 1500.0f;  // 2 registers
int32_t runSecs  = 7200;     // 2 registers
uint8_t nodeId   = 1;        // 1 register
bool    enabled  = true;     // 1 register, 0 or 1

beebus.addRegister(MB_HOLDING_REGISTER, 40, &levelHi, BEEBUS_T_F32);   // 40..41
beebus.addRegister(MB_HOLDING_REGISTER, 42, &runSecs, BEEBUS_T_I32);   // 42..43
beebus.addRegister(MB_HOLDING_REGISTER, 44, &nodeId,  BEEBUS_T_U8);    // 44
beebus.addRegister(MB_HOLDING_REGISTER, 45, &enabled, BEEBUS_T_BOOL);  // 45
```

| Type | Registers | Image |
| --- | --- | --- |
| `BEEBUS_T_BOOL` | 1 | 0 or 1. A write of any other number is TRUE. |
| `BEEBUS_T_I8`, `BEEBUS_T_U8` | 1 | Sign- or zero-extended. A write the type cannot hold is refused. |
| `BEEBUS_T_I16`, `BEEBUS_T_U16` | 1 | As is. |
| `BEEBUS_T_I32`, `BEEBUS_T_U32` | 2 | **High word first.** |
| `BEEBUS_T_F32` | 2 | IEEE-754 bits, high word first. |
| `BEEBUS_T_I64`, `BEEBUS_T_U64` | 4 | **High word first** ("ABCDEFGH"). |
| `BEEBUS_T_F64` | 4 | IEEE-754 binary64 bits, high word first. |

High word first is the word order a Modbus master calls "ABCD", "big-endian"
or "high word first" — the same order as the bytes inside a register and the
characters of a string. `bb_registersOf(type)` says how many registers a type
takes; `bb_valueRegister()` and `bb_setValueRegister()` give and take one
register of the image, so a master assembling a value uses the same rule.

- A value is **bound whole or not at all**: if any of its registers is taken,
  or it would run past 65535, nothing is bound.
- **Removing any of its registers removes the value.**
- A write of a number the type cannot hold — 300 into a `uint8_t`, -1 into a
  `uint8_t`, 200 into an `int8_t` — is **refused** with an exception rather than
  cut to a different number. In a multi-register write it refuses the whole
  request, which is rolled back.
- A two- or four-register value takes all its registers in one write
  (function 16). Writing only some of them changes that part of the value,
  which is what was asked for.
- A peer's write counts once against the value, on its first register.

### Which values a peer wrote

`registerWrites()` says *that* a range was written. `registerWritesSince()`
says *which* values were, so a settings block can check the one that changed:

```cpp
uint32_t seen = 0;
beebus.registerWritesSince(MB_HOLDING_REGISTER, 40, 6, seen);   // prime it

// every loop:
uint16_t first;
if (beebus.registerWritesSince(MB_HOLDING_REGISTER, 40, 6, seen, &first)) {
    validateSetting(first);      // the lowest address a peer wrote
}
```

It returns how many values were written since the caller last asked — a
two-register value or a string counts once — and advances `seen`. Each caller
keeps its own `seen`, so two parts of an application never take each other's
writes.

### Knowing when a read landed

```cpp
beebus.onReadComplete([](uint8_t node, uint16_t address, BeeBusResult result) {
    if (result == BeeBusResult::Immediate) {
        // The value is in your variable now
    }
});
```

**You need this more often than you expect.** Watching your own result variable
cannot distinguish a value that has not arrived yet from one that arrived
unchanged. This is especially true for strings, where an empty message and one
that has not landed look identical.

The handler applies to reads issued *after* it is set.

Likewise for confirmed writes:

```cpp
beebus.onWriteComplete([](uint8_t node, uint16_t address, BeeBusResult result) {
    if (result != BeeBusResult::Immediate) {
        // Timeout, NodeLost, Superseded, Refused, Cancelled ...
    }
});
```

### Being told when a peer writes to you

```cpp
beebus.onHregChanged(40, [](uint16_t address, int16_t value, uint8_t from) {
    applySetpoint(value);
});

beebus.onCoilChanged(0, [](uint16_t address, bool value, uint8_t from) {
    digitalWrite(PUMP_PIN, value);
});
```

Three things about these handlers:

- They are raised **only when the value actually changed**, and only for writes
  from a *peer*. A local `setHreg()` raises nothing — that is your application
  writing to itself.
- They are raised **once the whole request has been applied**, so a handler
  never sees a half-written block.
- They run **inside `loop()`**, while the frame that carried the write is being
  processed, so they delay this node's token pass. Keep them short. Set a flag
  and act on it in your main loop if there is real work to do.

## Strings

Text packed two characters to a register, big-endian, NUL terminated. It travels
as ordinary holding or input registers, so a Modbus gateway needs to know nothing
about it.

```cpp
char message[64];
beebus.addString(500, message, sizeof(message));      // Over holding registers
beebus.setString(500, "node 1 fault code 12");
```

| | |
| --- | --- |
| `addString` | Over holding registers — peers may write it |
| `addInputString` | Over input registers — peers may only read it |

A buffer of *capacity* bytes carries `capacity - 1` characters — its last byte
is always the terminator — and occupies `(capacity + 1) / 2` registers. With an
odd capacity the last register's second byte lies past the buffer: it reads as
0 and a write to it is dropped, so nothing outside your buffer is ever touched.
That is what lets a buffer of `n + 1` bytes carry all `n` characters in
`n / 2 + 1` registers.

### Reading a peer's string

```cpp
char buffer[64];
beebus.readString(NODE, 500, buffer, sizeof(buffer));
```

`size` is **how many bytes to read**, not how much room you happen to have. It
must fit your buffer *and* must not exceed the peer's field — reading `size / 2`
registers the peer has not bound is refused by the peer, exactly as a block read
past the end of a register range is.

Over 250 bytes is refused at the call, being past what one read carries.

Use `onReadComplete` to know when it landed. An empty message and one that has
not arrived look the same in the buffer.

### Writing a peer's string

```cpp
beebus.writeString(NODE, 500, "pump 2 overload", BeeBusDelivery::Confirmed);
```

Costs the length of the **message**, not the capacity of the field it lands in,
so publishing a short fault into a large field is cheap.

Refused with `InvalidArgument` beyond **245 characters**, which is what one
write carries. A split string could not be delivered atomically, and a fault
message that arrives in halves is worse than one that is refused.

A string raises **one** `onStringChanged` event for the whole message, not one
per register.

## Failsafe

What an output should do if the node commanding it stops answering.

```cpp
BeeBusConfig config;
config.enableFailSafe = true;
beebus.begin(&transport, NODE_ID, config);

beebus.setCoilFailsafe(0,  BeeBusFailsafe::ResetToValue, false);   // Stop the pump
beebus.setHregFailsafe(40, BeeBusFailsafe::ResetToValue, 500);     // Safe setpoint
beebus.setHregFailsafe(41, BeeBusFailsafe::HoldLastValue);         // Leave it
beebus.setStringFailsafe(500, BeeBusFailsafe::ResetToValue, "comms lost");
```

`HoldLastValue` is the **default**, for every address, always. Driving an output
is an action, and doing it unasked is its own hazard. A safe state is a property
of your plant, so it is stated per address.

The data map records which node last wrote each address. When that node leaves
the ring, every address it last wrote gets its stated action.

```cpp
beebus.onFailsafeApplied([](BeeBusRegisterType type, uint16_t address, uint8_t lostNode) {
    logAlarm("failsafe applied", address, lostNode);
});
```

**Register this handler.** Without it, an output driven to safe is
indistinguishable from one deliberately commanded, and that is a genuinely
dangerous ambiguity in an alarm log.

Failsafe is **off by default** and is a per-instance setting. Turn it on for
anything driving real outputs.

> **Failsafe is a register-layer feature.** It works because the data map
> records which node last wrote each address, and only writes that arrive
> through the register path are recorded. A **topic** write stores straight into
> the bound variable and records no writer, so `setCoilFailsafe` on a
> topic-driven output would never fire.
>
> So: command outputs that must fail safe over **coils and registers**, and use
> topics for measurements, status, and anything you want typed and
> quality-stamped.
>
> If you would rather command over topics, write the equivalent yourself — it is
> three lines, and `topicQuality()` is exactly the signal you need:
>
> ```cpp
> if (beebus.topicQuality(CONTROLLER, "enable") != BeeBusQuality::Good) {
>     driveOutputSafe();
> }
> ```

## Address map conventions

BeeBus does not impose one, but a consistent layout pays for itself:

```
Coils            0 –  99    Commands and enables
Discrete inputs  0 –  99    Status and alarm bits
Input registers 30 – 129    Measurements (read-only)
Holding regs    40 – 139    Setpoints and parameters
Holding regs   500 – 599    Strings
```

Two things worth doing regardless:

- **Leave gaps.** Reserving 20 addresses where you need 8 costs nothing and
  saves a renumbering exercise later.
- **Block-friendly grouping.** Put things that are read together next to each
  other, so one block read covers them. This is a real bandwidth decision on a
  radio link.

## Limits, collected

| | |
| --- | --- |
| Bound addresses per node | No limit but memory; every address of the four spaces |
| Registers per read | 125 |
| Registers per write | 123 |
| Coils per read | 2000 |
| Coils per write | 1968 |
| String read | 250 bytes |
| String write | 245 characters |
| Acknowledgements per section | 32 |
| Address range must not wrap | `start + quantity ≤ 65536` |

That last one matters: a block walked with 16-bit arithmetic that runs past
65535 reappears at address 0, and the addresses it would then reach are real,
mapped and completely unrelated. It is refused.

## Next

- [09 - Operations and framing](09-operations.md) — how these get batched onto the wire.
- [11 - Gateways](11-gateways.md) — putting a Modbus TCP gateway on a BeeBus network.
</content>
