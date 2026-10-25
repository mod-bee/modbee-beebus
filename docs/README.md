# BeeBus documentation

Everything here is written from the source: every constant, default and limit
quoted in these pages is the one the code uses.

## Where to start

**New to BeeBus?** Read [01 - Overview](01-overview.md), then work through
[02 - Getting started](02-getting-started.md). That is about an hour and you
will have two nodes exchanging live values.

**Coming from Modbus?** Read [01 - Overview](01-overview.md), then
[06 - The Modbus PDU](06-modbus-pdu.md). Your existing mental model transfers
almost completely; what changes is that there is no master.

**Designing a network?** Read [03 - How the ring works](03-how-the-ring-works.md)
and [13 - Applications](13-applications.md). Between them they cover how to lay
a network out and why.

**Implementing BeeBus somewhere else?** [04 - Frame format](04-frame-format.md),
[05 - The topic PDU](05-topic-pdu.md) and [06 - The Modbus PDU](06-modbus-pdu.md)
specify every byte on the wire.

## The whole set

| | |
| --- | --- |
| [01 - Overview](01-overview.md) | What BeeBus is, what it solves, and when not to use it |
| [02 - Getting started](02-getting-started.md) | A first network, one step at a time, with the reason for each step |
| [03 - How the ring works](03-how-the-ring-works.md) | Token passing, membership, joining, healing, and measured timing |
| [04 - Frame format](04-frame-format.md) | Every byte of a BeeBus frame |
| [05 - The topic PDU](05-topic-pdu.md) | The native data model: typed values, publish/subscribe, quality, buffering |
| [06 - The Modbus PDU](06-modbus-pdu.md) | The register data model and its wire format |
| [07 - Transports](07-transports.md) | RS485, UDP multicast, and writing your own |
| [08 - API reference](08-api-reference.md) | Every public call, what it returns, and when |
| [09 - Operations and framing](09-operations.md) | Queuing, superseding, acknowledgement, retries, and how a frame is packed |
| [10 - Configuration](10-configuration.md) | Settings, build flags, failsafe, and the clock |
| [11 - Gateways](11-gateways.md) | Joining two segments, and bridging to MQTT |
| [12 - Best practices](12-best-practices.md) | What to do, and what bites people |
| [13 - Applications](13-applications.md) | Six example networks, laid out and explained |

## The one idea worth carrying through all of it

You bind a variable. Somebody else asks for it. It arrives, and it keeps
arriving on the terms they asked for.

```cpp
float pressure = 0.0f;
beebus.addInputTopic("pressure", 1, &pressure);
```

That is the whole publisher. There is no polling loop, no message building, no
addressing scheme to invent, no broker to install and no master to elect. The
protocol works out who wants the value, when they are owed it, whose turn it is
to speak, and how long the network is currently taking to go round.

**You focus on the data. Not the protocol itself.**
</content>
