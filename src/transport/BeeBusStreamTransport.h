#pragma once

#include "../BeeBusTransport.h"

#if defined(BEEBUS_PLATFORM_ARDUINO)

#include <Arduino.h>
#include <Stream.h>

// Arduino serial transport. Wraps any Stream: HardwareSerial for RS485,
// SoftwareSerial, or anything else implementing the interface.
//
// The Stream must already be opened by the sketch; this class does not call
// begin() on it, because baud rate and pin assignment are board specific.
class BeeBusStreamTransport : public BeeBusTransport {
public:
    // The baud rate is the one thing about the line that cannot be worked out
    // from the bus: a link fast enough to deliver a whole frame between two
    // looks at it leaves no spacing to measure. The sketch already passed it to
    // Serial.begin(), so pass it here too and the node stops guessing: the
    // interframe gap, the response window and a joining node's announcement all
    // come off it. Left at zero the node falls back to measuring the line.
    explicit BeeBusStreamTransport(Stream* stream, uint32_t baudRate = 0)
        : _stream(stream), _baudRate(baudRate) {}

    bool begin() override { return _stream != nullptr; }

    void end() override { _stream = nullptr; }

    int available() override {
        return _stream ? _stream->available() : 0;
    }

    int read() override {
        return _stream ? _stream->read() : -1;
    }

    size_t write(const uint8_t* buffer, size_t length) override {
        return _stream ? _stream->write(buffer, length) : 0;
    }

    bool isConnected() const override { return _stream != nullptr; }

    uint32_t bitsPerSecond() const override { return _baudRate; }

private:
    Stream* _stream;
    uint32_t _baudRate;
};

#endif // BEEBUS_PLATFORM_ARDUINO
