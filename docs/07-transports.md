# 07 - Transports

A transport is the only part of BeeBus that knows what medium the bytes are
travelling over. The protocol core reads and writes bytes and never finds out.

```
                  Serial                       UDP
   Arduino   BeeBusStreamTransport      BeeBusArduinoUdpTransport
   Host      BeeBusHostSerialTransport  BeeBusHostUdpTransport
```

Any platform pairs with any medium. The same application code runs on all four.

## The interface

Six methods, and one optional. That is the whole contract (`BeeBusTransport.h`):

```cpp
class BeeBusTransport {
public:
    virtual ~BeeBusTransport() = default;
    virtual bool     begin() = 0;                                  // Open. False if unusable.
    virtual void     end() = 0;                                    // Close. Tolerates repeat calls.
    virtual int      available() = 0;                              // Bytes readable without blocking
    virtual int      read() = 0;                                   // Next byte, or -1. Never blocks.
    virtual size_t   write(const uint8_t* buffer, size_t length) = 0;  // One whole frame
    virtual bool     isConnected() const = 0;                      // Medium currently usable
    virtual uint32_t bitsPerSecond() const { return 0; }           // Line rate, 0 if not meaningful
};
```

**`read()` must never block.** `loop()` calls it in a tight drain and a blocking
read would stall the whole node.

**`write()` is handed one complete frame**, already escaped and flagged. A
partial write is treated as a failed send by the caller, so either take the
whole thing or take none of it.

**`bitsPerSecond()` is the line rate, where there is one.** A serial port knows
its baud rate; a network socket does not, and returns 0. It is the one property
of a line a node cannot measure for itself — a fast line delivers a whole frame
between two looks at it, leaving no spacing to read — so a transport that knows
it should say. The interframe gap, the response window and a joining node's
announcement are then sized from the rate rather than measured.

The caller keeps ownership of the transport, and it must outlive the
`BeeBusAPI` instance using it.

## Serial and RS485

The natural medium. Every node hears every frame because they share a pair of
wires, which is exactly what the protocol assumes.

### Arduino

The convenience overload wraps a `Stream` for you:

```cpp
Serial1.begin(115200);
beebus.begin(&Serial1, NODE_ID, BeeBusConfig(), 115200);   // The same baud rate
```

Pass the baud rate you gave `Serial1.begin()`, for the reason above. Leaving it
at 0 (the default) makes the node measure the line instead.

Or explicitly, which is what a gateway does:

```cpp
#include <transport/BeeBusStreamTransport.h>

BeeBusStreamTransport transport(&Serial1, 115200);
beebus.begin(&transport, NODE_ID);
```

Any `Stream` works — `HardwareSerial`, `SoftwareSerial`, anything implementing
the interface. **The sketch opens the stream**; the transport does not call
`begin()` on it, because baud rate and pin assignment are board specific.

ESP32, where the pins have to be named:

```cpp
Serial1.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
```

### Host

```cpp
#include <transport/BeeBusHostSerialTransport.h>

BeeBusSerialConfig config;
config.device   = "/dev/ttyUSB0";      // "COM3" on Windows
config.baudRate = 115200;

BeeBusHostSerialTransport transport(config);
if (!beebus.begin(&transport, NODE_ID)) {
    fprintf(stderr, "%s\n", transport.lastError());
}
```

The port is opened raw and non-blocking. Frames are 8N1, matching the Arduino
default. `baudRate` defaults to 115200 and is also what the transport reports
as its `bitsPerSecond()`.

**Virtual ports work identically.** A pair created with `socat` behaves exactly
as hardware does, which is what makes hardware-free testing possible:

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

### RS485 wiring, briefly

- A-to-A, B-to-B, one twisted pair. Get the polarity right; reversed A/B is
  silent failure.
- **120 Ω across the pair at each end of the run**, and only at the ends. This
  is the single most common cause of a link that mostly works.
- A ground reference between nodes. Isolated transceivers if the nodes sit on
  different earths.
- Daisy-chain, do not star. Stubs cause reflections.
- Idle-state biasing if your transceivers do not have it — a floating bus reads
  as noise and shows up as `noiseBytesDiscarded` climbing steadily.

### Choosing a baud rate

| Baud | Practical range | Notes |
| --- | --- | --- |
| 9600 | 1200 m | Long runs, electrically noisy sites |
| 19200 | 1200 m | Modbus RTU's traditional default |
| 115200 | 100–200 m | BeeBus's default. Good for a panel or a small plant. |
| 250000+ | Short | Only on clean, short, well-terminated cable |

**Every node on a segment must use the same baud rate.** A mismatch shows up as
`noiseBytesDiscarded` climbing without `framesReceived` moving.

Higher is not automatically better. A rotation on a five-node ring at 115200
takes single-digit milliseconds, which is faster than most process control needs,
and dropping to 19200 buys margin on a long or noisy run.

## UDP multicast

BeeBus is a token ring, so **every node must hear every frame on the segment.**
Multicast reproduces the RS485 shared-bus property that the protocol assumes.
Unicast would not work: nodes would not hear each other's token passes.

### Configuration

```cpp
struct BeeBusUdpConfig {
    const char* groupAddress   = "239.192.7.126";  // Organisation-local scope
    uint16_t    port           = 7126;
    uint8_t     ttl            = 1;                // Stay on this segment
    const char* interfaceAddress = nullptr;        // nullptr: let the OS choose
};
```

The default group sits in `239.192.0.0/14`, the IPv4 organisation-local scope,
which routers keep inside the site.

**TTL 1 is deliberate.** A bus segment means one segment. Raise it only if you
genuinely intend to span routed subnets, and read
[12 - Best practices](12-best-practices.md) first — a token ring across a WAN
link is usually a worse idea than a gateway.

### Host

```cpp
#include <transport/BeeBusHostUdpTransport.h>

BeeBusUdpConfig config;
BeeBusHostUdpTransport transport(config);
beebus.begin(&transport, NODE_ID);
```

Two sockets are used deliberately. The receive socket binds the group port with
address reuse, so **several nodes can run on one machine** — which is how the
examples work. The send socket is connected to the group, which lets the OS
report the exact source address and port our own datagrams will carry, so
loopback echo can be filtered out precisely.

### Arduino

```cpp
#include <transport/BeeBusArduinoUdpTransport.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
BeeBusUdpConfig config;
BeeBusArduinoUdpTransport transport(&udp, config);

void setup() {
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) delay(250);

    // Join the group before handing the socket over.
    IPAddress group;
    group.fromString(config.groupAddress);
    udp.beginMulticast(group, config.port);

    beebus.begin(&transport, NODE_ID);
    beebus.connect();
}
```

`beginMulticast()` is not part of the base Arduino `UDP` class, so the sketch
joins the group and hands over a socket that is already joined. `WiFiUDP` and
`EthernetUDP` both expose it on ESP32 and ESP8266.

Any UDP implementation works — `WiFiUDP`, `EthernetUDP`, or a wrapper of your
own.

### Datagram boundaries

Datagram transports present a byte stream by draining exactly **one datagram at
a time**, so frames never straddle the read buffer. Beyond that, the framer
resynchronises on the flag byte, so a datagram boundary needs no special
handling anywhere in the core.

### What UDP changes

| | |
| --- | --- |
| **Faster** | Rotations in the low milliseconds, and no half-duplex turnaround. |
| **Lossier than you think** | Multicast on a busy switch is not lossless. BeeBus's retry and gap detection cover it, but choose deadbands accordingly. |
| **Switch behaviour matters** | IGMP snooping on a managed switch decides whether your frames reach every node. Get it wrong and you have a partitioned ring. |
| **No physical layer to blame** | `crcErrors` on UDP means something odd. `framesReceived` not rising means multicast is not being delivered. |
| **Anyone on the LAN can join** | There is no authentication. Keep it on a network you control, or a VLAN of its own. |

## Radio links

There is no radio transport class, and there does not need to be one. Modules
like the Ebyte E52 series present a serial interface, so `BeeBusStreamTransport`
or `BeeBusHostSerialTransport` is the transport — the radio is simply a slow,
lossy wire.

That is the whole point of measuring timing rather than configuring it. A node
behind a radio has a hop time ten or a hundred times its neighbours', BeeBus
measures it, gives that link the timeouts it deserves, and gives that node the
token less often so it does not pace the whole segment.

What you should do differently on radio:

- **Loose deadbands and long intervals.** Every byte is expensive.
- **Fetch topic lists without names**, or not at all. Bind by ID and skip the
  commissioning traffic.
- **Keep topic IDs under 128** so they encode in one byte.
- **Confirmed writes for commands, unconfirmed for telemetry.** The next reading
  is along shortly; an ack per reading is airtime you cannot spare.
- **Buffer the topics that matter** so a fade costs you a delay, not data.
- **Consider a gateway instead.** A radio segment carrying three nodes and a
  wired segment carrying twelve, joined by a gateway, is nearly always better
  than fifteen nodes on one mixed ring. See [11 - Gateways](11-gateways.md).

## Writing your own transport

Everything the core needs is in those six methods. A transport for anything that
moves bytes — LoRaWAN, CAN with a framing shim, a mesh module, a test double —
is a small class.

```cpp
#include <BeeBusTransport.h>

class MyTransport : public BeeBusTransport {
public:
    bool begin() override {
        return openTheMedium();
    }

    void end() override {
        closeTheMedium();               // Must tolerate being called twice
    }

    int available() override {
        return bytesWaiting();          // 0 is fine, never block
    }

    int read() override {
        return bytesWaiting() ? nextByte() : -1;
    }

    size_t write(const uint8_t* buffer, size_t length) override {
        return sendAllOrNothing(buffer, length);   // Return length, or 0
    }

    bool isConnected() const override {
        return _open;
    }
};
```

Four rules:

1. **Never block in `read()` or `available()`.**
2. **Write the whole frame or none of it.** A partial write is a failed send.
3. **Every node must hear every frame.** If your medium is point-to-point, you
   need a broadcast or multicast mode, or the ring cannot form.
4. **Do not reorder or fragment.** The framer resynchronises on the flag byte, so
   a lost frame is survivable, but a transport that delivers halves of frames
   out of order is not a bus.

If your medium loops your own transmissions back, that is handled: the core
remembers the CRC and length of the last frame it sent and, within one interframe
gap, recognises the echo rather than mistaking it for another node using its
address.

## Two transports in one node

A gateway runs two `BeeBusAPI` instances, one per transport, and calls `loop()`
on both:

```cpp
BeeBusAPI serialSide, udpSide;

void setup() {
    // ... open Serial1 and join the multicast group first
    serialSide.begin(&Serial1, NODE_ID, BeeBusConfig(), 115200);
    udpSide.begin(&udpTransport, NODE_ID);
    serialSide.connect();
    udpSide.connect();
}

void loop() {
    serialSide.loop();
    udpSide.loop();
}
```

Two separate rings, two address spaces, two independently measured timings. The
node may carry the **same ID on both**, because neither ring can hear the other.

See [11 - Gateways](11-gateways.md).

## Timing across media

`interframeGapUs` is the minimum silence a node leaves before transmitting, and
it is **measured** like everything else: two byte times taken from the line, and
never shorter than the interval this node looks at the bus in.

Pin it only where the hardware needs longer than the line alone accounts for — a
transceiver slow to turn around. A pinned value is a *floor*, not a replacement,
and is still raised if the line turns out slower than it assumed.

Consider pinning a floor for:

- Slow transceivers that need turnaround time
- Half-duplex radios with a transmit/receive switching delay

A pinned value can only raise the gap: it never takes it below what the line
itself needs.

Everything else about timing adapts on its own. See
[03 - How the ring works](03-how-the-ring-works.md).
</content>
