#pragma once

#include "../BeeBusTransport.h"
#include "../BeeBusTypes.h"

#if defined(BEEBUS_PLATFORM_ARDUINO)

#include <Arduino.h>
#include <Udp.h>
#include <IPAddress.h>

#include "BeeBusUdpConfig.h"

// Arduino UDP transport for Ethernet or WiFi links.
//
// Takes any UDP implementation (WiFiUDP, EthernetUDP, AsyncUDP wrappers) so it
// works across boards. The token ring requires every node to hear every frame,
// so frames go to a multicast group that all nodes join.
//
// beginMulticast() is not part of the base Arduino UDP class. Pass a UDP object
// already joined to the group, or use a subclass that exposes it (WiFiUDP and
// EthernetUDP both do on ESP32/ESP8266).
class BeeBusArduinoUdpTransport : public BeeBusTransport {
public:
    BeeBusArduinoUdpTransport(UDP* udp, const BeeBusUdpConfig& config)
        : _udp(udp), _config(config), _rxLength(0), _rxPos(0), _started(false) {}

    bool begin() override {
        if (!_udp) return false;
        _group.fromString(_config.groupAddress);
        _rxLength = 0;
        _rxPos = 0;
        _started = true;
        return true;
    }

    void end() override {
        if (_udp && _started) _udp->stop();
        _started = false;
        _rxLength = 0;
        _rxPos = 0;
    }

    int available() override {
        if (_rxPos < _rxLength) {
            return (int)(_rxLength - _rxPos);
        }
        receiveNextDatagram();
        return (int)(_rxLength - _rxPos);
    }

    int read() override {
        if (available() <= 0) return -1;
        return _rxBuffer[_rxPos++];
    }

    size_t write(const uint8_t* buffer, size_t length) override {
        if (!_udp || !_started || !buffer || length == 0) return 0;
        if (_udp->beginPacket(_group, _config.port) != 1) return 0;
        const size_t written = _udp->write(buffer, length);
        if (_udp->endPacket() != 1) return 0;
        return written;
    }

    bool isConnected() const override { return _udp != nullptr && _started; }

private:
    // Drains exactly one datagram so frames never straddle the read buffer.
    void receiveNextDatagram() {
        _rxLength = 0;
        _rxPos = 0;
        if (!_udp || !_started) return;

        const int size = _udp->parsePacket();
        if (size <= 0) return;

        // Longer than any frame: not BeeBus. Dropped whole, since a frame cut
        // short would only fail its CRC.
        if (size > BEEBUS_MAX_WIRE_FRAME) {
            _udp->flush();
            return;
        }
        const int got = _udp->read(_rxBuffer, size);
        if (got > 0) _rxLength = (uint16_t)got;

        _udp->flush();
    }

    UDP* _udp;
    BeeBusUdpConfig _config;
    IPAddress _group;

    uint8_t _rxBuffer[BEEBUS_MAX_WIRE_FRAME];   // a whole frame as it travels, escapes included
    uint16_t _rxLength;
    uint16_t _rxPos;
    bool _started;
};

#endif // BEEBUS_PLATFORM_ARDUINO
