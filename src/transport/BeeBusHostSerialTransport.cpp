#include "BeeBusHostSerialTransport.h"

#if defined(BEEBUS_PLATFORM_HOST)

#include <cstdio>
#include <cstring>

#if defined(_WIN32)
    #include <windows.h>
#else
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
    #include <termios.h>
    #include <sys/ioctl.h>
#endif

namespace {

#if !defined(_WIN32)
// Maps a numeric baud rate onto the termios constant. Returns false when the
// rate has no portable constant, rather than silently picking a wrong speed.
bool toSpeed(uint32_t baud, speed_t& out) {
    switch (baud) {
        case 1200:    out = B1200;    return true;
        case 2400:    out = B2400;    return true;
        case 4800:    out = B4800;    return true;
        case 9600:    out = B9600;    return true;
        case 19200:   out = B19200;   return true;
        case 38400:   out = B38400;   return true;
        case 57600:   out = B57600;   return true;
        case 115200:  out = B115200;  return true;
        case 230400:  out = B230400;  return true;
#ifdef B460800
        case 460800:  out = B460800;  return true;
#endif
#ifdef B921600
        case 921600:  out = B921600;  return true;
#endif
        default: return false;
    }
}
#endif

} // namespace

BeeBusHostSerialTransport::BeeBusHostSerialTransport(const BeeBusSerialConfig& config)
    : _config(config),
      _device(config.device ? config.device : ""),
      _handle(INVALID_HANDLE),
      _peeked(-1) {}

BeeBusHostSerialTransport::~BeeBusHostSerialTransport() {
    end();
}

bool BeeBusHostSerialTransport::fail(const char* what) {
    _lastError = what;
    end();
    return false;
}

bool BeeBusHostSerialTransport::failOs(const char* what) {
#if defined(_WIN32)
    const unsigned long code = (unsigned long)GetLastError();
    char words[32];
    snprintf(words, sizeof(words), "error %lu", (unsigned long)code);
    _lastError = std::string(what) + ": " + words;
#else
    const int code = errno;
    _lastError = std::string(what) + ": " + std::strerror(code);
#endif
    end();
    return false;
}

bool BeeBusHostSerialTransport::begin() {
    end();
    _lastError.clear();

    if (_device.empty()) {
        return fail("no serial device specified");
    }

#if defined(_WIN32)
    // The \\.\ prefix is required for COM10 and above.
    std::string path = _device;
    if (path.compare(0, 4, "\\\\.\\") != 0) {
        path = "\\\\.\\" + path;
    }

    const HANDLE handle = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                                      0, NULL, OPEN_EXISTING, 0, NULL);
    if (handle == INVALID_HANDLE_VALUE) {
        return failOs("could not open serial device");
    }
    _handle = (handle_t)handle;
#else
    const int fd = ::open(_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return failOs("could not open serial device");
    }
    _handle = (handle_t)fd;
#endif

    if (!configurePort()) {
        return false;
    }

    _peeked = -1;
    return true;
}

bool BeeBusHostSerialTransport::configurePort() {
#if defined(_WIN32)
    DCB dcb;
    std::memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState((HANDLE)_handle, &dcb)) {
        return failOs("could not read port state");
    }

    dcb.BaudRate = _config.baudRate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;

    if (!SetCommState((HANDLE)_handle, &dcb)) {
        return failOs("could not apply port settings");
    }

    // All-zero timeouts make reads return whatever is buffered, immediately.
    COMMTIMEOUTS timeouts;
    std::memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = MAXDWORD;
    if (!SetCommTimeouts((HANDLE)_handle, &timeouts)) {
        return failOs("could not apply port timeouts");
    }

    PurgeComm((HANDLE)_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return true;
#else
    speed_t speed;
    if (!toSpeed(_config.baudRate, speed)) {
        return fail("unsupported baud rate");
    }

    termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr((int)_handle, &tty) != 0) {
        return failOs("could not read port state");
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);   // Ignore modem lines, enable receiver
    tty.c_cflag &= ~CSTOPB;            // 1 stop bit
    tty.c_cflag &= ~PARENB;            // No parity
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                // 8 data bits
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;           // No hardware flow control
#endif

    // Fully non-blocking: return immediately with whatever is buffered.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr((int)_handle, TCSANOW, &tty) != 0) {
        return failOs("could not apply port settings");
    }

    tcflush((int)_handle, TCIOFLUSH);
    return true;
#endif
}

void BeeBusHostSerialTransport::end() {
    if (_handle != INVALID_HANDLE) {
#if defined(_WIN32)
        CloseHandle((HANDLE)_handle);
#else
        ::close((int)_handle);
#endif
        _handle = INVALID_HANDLE;
    }
    _peeked = -1;
}

int BeeBusHostSerialTransport::available() {
    if (_handle == INVALID_HANDLE) return 0;
    if (_peeked >= 0) return 1;

    _peeked = read();
    return _peeked >= 0 ? 1 : 0;
}

int BeeBusHostSerialTransport::read() {
    if (_peeked >= 0) {
        const int value = _peeked;
        _peeked = -1;
        return value;
    }
    if (_handle == INVALID_HANDLE) return -1;

    uint8_t byte = 0;
#if defined(_WIN32)
    DWORD got = 0;
    if (!ReadFile((HANDLE)_handle, &byte, 1, &got, NULL) || got != 1) {
        return -1;
    }
#else
    const ssize_t got = ::read((int)_handle, &byte, 1);
    if (got != 1) {
        return -1;
    }
#endif
    return (int)byte;
}

size_t BeeBusHostSerialTransport::write(const uint8_t* buffer, size_t length) {
    if (_handle == INVALID_HANDLE || !buffer || length == 0) return 0;

    // A non-blocking descriptor answers EAGAIN while the driver's buffer is
    // full, and it empties at the line rate, so waiting for room is waiting for
    // the wire. Bounded by how long these bytes take to leave at the configured
    // rate, with room for the buffer that was already ahead of them: past that
    // the device has stopped draining, and this runs inside the protocol's own
    // loop, where waiting stops the node receiving, timing and passing the
    // token. A short count says the frame did not go out, which the caller
    // already treats as a frame not sent.
    const uint32_t bps = _config.baudRate != 0 ? _config.baudRate : 9600;
    const bb_time_t allowanceUs =
        (bb_time_t)((10ull * 1000000ull * length) / bps) * 2 + 1000;
    const bb_time_t startedUs = bb_micros();

    size_t total = 0;
    while (total < length) {
#if defined(_WIN32)
        DWORD written = 0;
        if (!WriteFile((HANDLE)_handle, buffer + total,
                       (DWORD)(length - total), &written, NULL)) {
            break;
        }
        if (written == 0) break;
        total += written;
#else
        const ssize_t written = ::write((int)_handle, buffer + total, length - total);
        if (written < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (bb_elapsed(startedUs, bb_micros()) < allowanceUs) continue;
            }
            break;
        }
        if (written == 0) break;
        total += (size_t)written;
#endif
    }
    return total;
}

bool BeeBusHostSerialTransport::isConnected() const {
    return _handle != INVALID_HANDLE;
}

#endif // BEEBUS_PLATFORM_HOST
