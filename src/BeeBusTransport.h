#pragma once

#include "BeeBusPlatform.h"

// --- Abstract transport interface ---

// The single point of contact between the protocol core and the outside world.
// The core reads and writes bytes; it never learns which medium carries them.
//
// Implementations live in src/transport/ and are selected by the application,
// so any platform can pair with any medium:
//
//                  Serial                          UDP
//   Arduino    BeeBusStreamTransport      BeeBusArduinoUdpTransport
//   Host       BeeBusHostSerialTransport  BeeBusHostUdpTransport
//
// Datagram transports present a byte stream by draining one datagram at a time.
// The framer resynchronises on FLAG, so a datagram boundary needs no special
// handling in the core.
class BeeBusTransport {
public:
    virtual ~BeeBusTransport() = default;

    // Open the underlying medium. Returns false if it cannot be used.
    virtual bool begin() = 0;

    // Close the medium and release resources. Must tolerate repeat calls.
    virtual void end() = 0;

    // Bytes that can be read without blocking.
    virtual int available() = 0;

    // Next byte, or -1 if none is available. Never blocks.
    virtual int read() = 0;

    // Write one whole frame. Returns bytes accepted; partial writes are treated
    // as a failed send by the caller.
    virtual size_t write(const uint8_t* buffer, size_t length) = 0;

    // Whether the medium is currently usable.
    virtual bool isConnected() const = 0;

    // Bits per second the link clocks at, or 0 where that is not a meaningful
    // question. A serial port knows it and a network socket does not. Given
    // rather than measured: byte timing is read from the spacing of arrivals,
    // and a fast line delivers a whole frame between two looks at it, leaving
    // no spacing to read. The one thing that cannot be worked out from the bus
    // is therefore the one thing the transport already knows.
    virtual uint32_t bitsPerSecond() const { return 0; }
};
