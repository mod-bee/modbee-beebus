#include "BeeBusHostUdpTransport.h"

#if defined(BEEBUS_PLATFORM_HOST)

#include <cstdio>
#include <cstring>

#if defined(_WIN32)
    #include <winsock2.h>
    #include <ws2tcpip.h>
    typedef int socklen_t;
    #define MB_CLOSE_SOCKET closesocket
    #define MB_WOULD_BLOCK() (WSAGetLastError() == WSAEWOULDBLOCK)
    typedef char sockopt_t;
#else
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #define MB_CLOSE_SOCKET ::close
    #define MB_WOULD_BLOCK() (errno == EAGAIN || errno == EWOULDBLOCK)
    typedef int sockopt_t;
#endif

namespace {

#if defined(_WIN32)
// Winsock needs process-wide startup. Tied to a static so it happens once.
struct WinsockGuard {
    bool ok;
    WinsockGuard() {
        WSADATA data;
        ok = (WSAStartup(MAKEWORD(2, 2), &data) == 0);
    }
    ~WinsockGuard() { if (ok) WSACleanup(); }
};

bool ensureWinsock() {
    static WinsockGuard guard;
    return guard.ok;
}
#else
bool ensureWinsock() { return true; }
#endif

bool setNonBlocking(intptr_t handle) {
#if defined(_WIN32)
    u_long mode = 1;
    return ioctlsocket((SOCKET)handle, FIONBIO, &mode) == 0;
#else
    const int flags = fcntl((int)handle, F_GETFL, 0);
    return flags != -1 && fcntl((int)handle, F_SETFL, flags | O_NONBLOCK) != -1;
#endif
}

} // namespace

BeeBusHostUdpTransport::BeeBusHostUdpTransport(const BeeBusUdpConfig& config)
    : _config(config),
      _rxSocket(INVALID_HANDLE),
      _txSocket(INVALID_HANDLE),
      _selfAddress(0),
      _selfPort(0),
      _rxLength(0),
      _rxPos(0) {}

BeeBusHostUdpTransport::~BeeBusHostUdpTransport() {
    end();
}

bool BeeBusHostUdpTransport::fail(const char* what) {
    _lastError = what;
    end();
    return false;
}

bool BeeBusHostUdpTransport::failOs(const char* what) {
#if defined(_WIN32)
    const int code = WSAGetLastError();
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

bool BeeBusHostUdpTransport::begin() {
    end();
    _lastError.clear();

    if (!ensureWinsock()) {
        return fail("winsock startup failed");
    }
    if (inet_addr(_config.groupAddress) == INADDR_NONE) {
        return fail("invalid multicast group address");
    }
    if (!openReceiveSocket()) return false;
    if (!openSendSocket()) return false;

    _rxLength = 0;
    _rxPos = 0;
    return true;
}

bool BeeBusHostUdpTransport::openReceiveSocket() {
    _rxSocket = (socket_t)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_rxSocket == INVALID_HANDLE) {
        return failOs("could not create receive socket");
    }

    // Address reuse lets several nodes share the group port on one machine.
    const sockopt_t reuse = 1;
    setsockopt((int)_rxSocket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#if defined(SO_REUSEPORT)
    setsockopt((int)_rxSocket, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    sockaddr_in local;
    std::memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(_config.port);

    if (bind((int)_rxSocket, (sockaddr*)&local, sizeof(local)) != 0) {
        return failOs("could not bind group port");
    }

    ip_mreq group;
    std::memset(&group, 0, sizeof(group));
    group.imr_multiaddr.s_addr = inet_addr(_config.groupAddress);
    group.imr_interface.s_addr = _config.interfaceAddress
        ? inet_addr(_config.interfaceAddress)
        : htonl(INADDR_ANY);

    if (setsockopt((int)_rxSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                   (const sockopt_t*)&group, sizeof(group)) != 0) {
        return failOs("could not join multicast group");
    }

    if (!setNonBlocking(_rxSocket)) {
        return failOs("could not set receive socket non-blocking");
    }
    return true;
}

bool BeeBusHostUdpTransport::openSendSocket() {
    _txSocket = (socket_t)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_txSocket == INVALID_HANDLE) {
        return failOs("could not create send socket");
    }

    const sockopt_t ttl = (sockopt_t)_config.ttl;
    setsockopt((int)_txSocket, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

    // Loopback on, so nodes sharing a machine hear one another. Our own frames
    // are filtered by source address in receiveNextDatagram().
    const sockopt_t loop = 1;
    setsockopt((int)_txSocket, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

    if (_config.interfaceAddress) {
        in_addr iface;
        iface.s_addr = inet_addr(_config.interfaceAddress);
        if (setsockopt((int)_txSocket, IPPROTO_IP, IP_MULTICAST_IF,
                       (const sockopt_t*)&iface, sizeof(iface)) != 0) {
            return failOs("could not select multicast interface");
        }
    }

    sockaddr_in target;
    std::memset(&target, 0, sizeof(target));
    target.sin_family = AF_INET;
    target.sin_addr.s_addr = inet_addr(_config.groupAddress);
    target.sin_port = htons(_config.port);

    // Connecting fixes the destination and makes the kernel commit to a source
    // address, which getsockname() then reports for the echo filter.
    if (connect((int)_txSocket, (sockaddr*)&target, sizeof(target)) != 0) {
        return failOs("could not connect send socket to group");
    }

    sockaddr_in bound;
    socklen_t boundLen = sizeof(bound);
    std::memset(&bound, 0, sizeof(bound));
    if (getsockname((int)_txSocket, (sockaddr*)&bound, &boundLen) != 0) {
        return failOs("could not resolve local send address");
    }
    _selfAddress = bound.sin_addr.s_addr;
    _selfPort = bound.sin_port;

    if (!setNonBlocking(_txSocket)) {
        return fail("could not set send socket non-blocking");
    }
    return true;
}

void BeeBusHostUdpTransport::end() {
    if (_rxSocket != INVALID_HANDLE) {
        MB_CLOSE_SOCKET((int)_rxSocket);
        _rxSocket = INVALID_HANDLE;
    }
    if (_txSocket != INVALID_HANDLE) {
        MB_CLOSE_SOCKET((int)_txSocket);
        _txSocket = INVALID_HANDLE;
    }
    _selfAddress = 0;
    _selfPort = 0;
    _rxLength = 0;
    _rxPos = 0;
}

int BeeBusHostUdpTransport::available() {
    if (_rxPos < _rxLength) {
        return (int)(_rxLength - _rxPos);
    }
    receiveNextDatagram();
    return (int)(_rxLength - _rxPos);
}

int BeeBusHostUdpTransport::read() {
    if (available() <= 0) return -1;
    return _rxBuffer[_rxPos++];
}

// Takes one datagram at a time so frames never straddle the read buffer.
void BeeBusHostUdpTransport::receiveNextDatagram() {
    _rxLength = 0;
    _rxPos = 0;
    if (_rxSocket == INVALID_HANDLE) return;

    for (;;) {
        sockaddr_in from;
        socklen_t fromLen = sizeof(from);
        std::memset(&from, 0, sizeof(from));

        const int received = (int)recvfrom((int)_rxSocket, (char*)_rxBuffer,
                                           (int)sizeof(_rxBuffer), 0,
                                           (sockaddr*)&from, &fromLen);
        if (received <= 0) {
            return; // Nothing pending, or a datagram we cannot use.
        }

        // Skip our own multicast echo.
        if (from.sin_addr.s_addr == _selfAddress && from.sin_port == _selfPort) {
            continue;
        }

        // Longer than any frame: not BeeBus. Dropped whole, since a frame cut
        // short would only fail its CRC and take the next one's start with it.
        if (received > BEEBUS_MAX_WIRE_FRAME) {
            continue;
        }

        _rxLength = (uint16_t)received;
        return;
    }
}

size_t BeeBusHostUdpTransport::write(const uint8_t* buffer, size_t length) {
    if (_txSocket == INVALID_HANDLE || !buffer || length == 0) return 0;

    const int sent = (int)send((int)_txSocket, (const char*)buffer, (int)length, 0);
    if (sent < 0) {
        if (!MB_WOULD_BLOCK()) _lastError = "datagram send failed";
        return 0;
    }
    return (size_t)sent;
}

bool BeeBusHostUdpTransport::isConnected() const {
    return _rxSocket != INVALID_HANDLE && _txSocket != INVALID_HANDLE;
}

#endif // BEEBUS_PLATFORM_HOST
