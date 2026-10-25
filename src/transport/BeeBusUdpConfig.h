#pragma once

#include "../BeeBusPlatform.h"

// Shared settings for every UDP transport, host and Arduino alike.
//
// BeeBus is a token ring: each node must hear every frame on the segment.
// Multicast reproduces the RS485 shared-bus property that the protocol assumes,
// so all nodes join one group and send to it.
struct BeeBusUdpConfig {
    // Group all nodes join. Default sits in the IPv4 organisation-local scope
    // (239.192.0.0/14), which routers keep inside the site.
    const char* groupAddress = "239.192.7.126";

    // Port all nodes bind and send to.
    uint16_t port = 7126;

    // Multicast hop limit. 1 keeps frames on the local segment, which is what a
    // bus segment means. Raise it only to span routed subnets.
    uint8_t ttl = 1;

    // Local interface to send and receive on. nullptr lets the OS choose, which
    // is usually right on single-homed hosts and Docker containers.
    const char* interfaceAddress = nullptr;
};
