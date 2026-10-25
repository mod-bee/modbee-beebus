#pragma once

#include "../BeeBusTransport.h"

#if defined(BEEBUS_PLATFORM_HOST)

#include <string>

// Serial port settings. BeeBus frames are 8N1, matching the Arduino default.
struct BeeBusSerialConfig {
    // Device path. "/dev/ttyUSB0" or a socat PTY on POSIX, "COM3" on Windows.
    const char* device = nullptr;

    uint32_t baudRate = 115200;
};

// Serial transport for Linux, macOS and Windows.
//
// The port is opened raw and non-blocking, so read() returns immediately when
// no bytes are waiting. Virtual ports created by socat behave identically to
// hardware ones here, which is what makes hardware-free testing work.
class BeeBusHostSerialTransport : public BeeBusTransport {
public:
    typedef intptr_t handle_t;
    static const handle_t INVALID_HANDLE = -1;

    explicit BeeBusHostSerialTransport(const BeeBusSerialConfig& config);
    ~BeeBusHostSerialTransport() override;

    BeeBusHostSerialTransport(const BeeBusHostSerialTransport&) = delete;
    BeeBusHostSerialTransport& operator=(const BeeBusHostSerialTransport&) = delete;

    bool begin() override;
    void end() override;
    int available() override;
    int read() override;
    size_t write(const uint8_t* buffer, size_t length) override;
    bool isConnected() const override;
    uint32_t bitsPerSecond() const override { return _config.baudRate; }

    // Reason the last begin() failed. Empty when no error.
    const char* lastError() const { return _lastError.c_str(); }

private:
    bool configurePort();
    bool fail(const char* what);
    // The same, for a failed system call: the operating system's own words
    // follow, "could not open serial device: No such file or directory", read
    // before anything else can overwrite them.
    bool failOs(const char* what);

    BeeBusSerialConfig _config;
    std::string _device;
    handle_t _handle;

    // One byte of lookahead, so available() can report a byte that read()
    // has already pulled from the OS buffer.
    int _peeked;

    std::string _lastError;
};

#endif // BEEBUS_PLATFORM_HOST
