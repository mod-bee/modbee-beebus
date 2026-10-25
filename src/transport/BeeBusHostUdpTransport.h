#pragma once

#include "../BeeBusTransport.h"

#if defined(BEEBUS_PLATFORM_HOST)

#include "../BeeBusTypes.h"
#include "BeeBusUdpConfig.h"

#include <string>

// UDP multicast transport for Linux, macOS and Windows.
//
// Two sockets are used deliberately. The receive socket binds the group port
// with address reuse, so several nodes can run on one machine. The send socket
// is connected to the group, which lets the OS report the exact source address
// and port our own datagrams will carry. Multicast loopback stays enabled so
// same-host nodes hear each other, and our own frames are filtered out by
// matching that address pair.
class BeeBusHostUdpTransport : public BeeBusTransport {
public:
    typedef intptr_t socket_t;
    static const socket_t INVALID_HANDLE = -1;

    explicit BeeBusHostUdpTransport(const BeeBusUdpConfig& config = BeeBusUdpConfig());
    ~BeeBusHostUdpTransport() override;

    BeeBusHostUdpTransport(const BeeBusHostUdpTransport&) = delete;
    BeeBusHostUdpTransport& operator=(const BeeBusHostUdpTransport&) = delete;

    bool begin() override;
    void end() override;
    int available() override;
    int read() override;
    size_t write(const uint8_t* buffer, size_t length) override;
    bool isConnected() const override;

    // Reason the last begin() or write() failed. Empty when no error.
    const char* lastError() const { return _lastError.c_str(); }

private:
    bool openReceiveSocket();
    bool openSendSocket();
    void receiveNextDatagram();
    bool fail(const char* what);
    // The same, for a failed system call: the operating system's own words
    // follow, "could not bind group port: Address already in use", read
    // before anything else can overwrite them.
    bool failOs(const char* what);

    BeeBusUdpConfig _config;
    socket_t _rxSocket;
    socket_t _txSocket;

    // Source address our own datagrams carry, used to drop loopback echo.
    uint32_t _selfAddress;
    uint16_t _selfPort;

    // A whole frame as it travels, escapes included, and one byte more: a
    // datagram that fills it is longer than any frame and is dropped.
    uint8_t _rxBuffer[BEEBUS_MAX_WIRE_FRAME + 1];
    uint16_t _rxLength;
    uint16_t _rxPos;

    std::string _lastError;
};

#endif // BEEBUS_PLATFORM_HOST
