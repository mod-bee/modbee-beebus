// A gateway between an RS485 segment and a UDP segment.
//
// Usage:
//   beebus_gateway_node <device> <node-id> <field-node-id> [baud] [group] [port]
//
//   ./beebus_gateway_node /dev/ttyUSB0 10 1
//
// The two segments are separate BeeBus networks: separate token rings, separate
// address spaces, separate rotation times. This node is a member of both, so it
// runs two BeeBusAPI instances, one per transport, and calls loop() on each.
//
// A value crosses because the same variable is bound into both maps. Nothing
// else is forwarded, and neither segment learns anything about the other's
// members.
//
//   upstream    registered and subscribed on the serial side, owned on the UDP
//               side from the same variable
//   downstream  owned on the UDP side so it can be written, owned on the serial
//               side so it can be subscribed to, from the same variable
//
// Neither direction needs forwarding code: the publish scan on the far side
// sees the variable change and reports it.

#include <BeeBusGlobal.h>
#include <transport/BeeBusHostSerialTransport.h>
#include <transport/BeeBusHostUdpTransport.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {

volatile std::sig_atomic_t g_running = 1;

void onSignal(int) { g_running = 0; }

// Topic IDs. Each segment numbers its own, so these need not agree with the
// field node's beyond the one topic that is registered from it.
const uint16_t TOPIC_VALUE   = 1;
const uint16_t TOPIC_QUALITY = 2;
const uint16_t TOPIC_COMMAND = 10;

} // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr,
                     "usage: %s <device> <node-id> <field-node-id> [baud] [group] [port]\n",
                     argv[0]);
        return 2;
    }

    BeeBusSerialConfig serialConfig;
    serialConfig.device = argv[1];
    serialConfig.baudRate = (argc > 4) ? (uint32_t)std::strtoul(argv[4], nullptr, 10) : 115200;

    const uint8_t nodeID = (uint8_t)std::strtoul(argv[2], nullptr, 10);
    const uint8_t fieldNodeID = (uint8_t)std::strtoul(argv[3], nullptr, 10);

    BeeBusUdpConfig udpConfig;
    if (argc > 5) udpConfig.groupAddress = argv[5];
    if (argc > 6) udpConfig.port = (uint16_t)std::strtoul(argv[6], nullptr, 10);

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    BeeBusHostSerialTransport serialTransport(serialConfig);
    BeeBusHostUdpTransport udpTransport(udpConfig);

    // One instance per segment. The config belongs to the instance, so the two
    // sides measure their own timing and never share a timeout.
    BeeBusAPI serialSide, udpSide;

    if (!serialSide.begin(&serialTransport, nodeID)) {
        std::fprintf(stderr, "cannot open %s: %s\n",
                     serialConfig.device, serialTransport.lastError());
        return 1;
    }
    if (!udpSide.begin(&udpTransport, nodeID)) {
        std::fprintf(stderr, "cannot open %s:%u: %s\n",
                     udpConfig.groupAddress, udpConfig.port, udpTransport.lastError());
        return 1;
    }

    // --- The variables that cross ---
    float analogValue = 0.0f;
    uint8_t analogQuality = (uint8_t)BeeBusQuality::Unknown;
    int16_t command = 0;
    int16_t sharedRegister = 0;

    // Upstream: the field node's topic lands here, and the UDP side publishes
    // the same variable to whoever subscribes.
    serialSide.registerTopic(fieldNodeID, "analogValue", TOPIC_VALUE, &analogValue);
    udpSide.addInputTopic("analogValue", TOPIC_VALUE, &analogValue);

    // Quality is the one thing a shared variable cannot carry. The UDP side sees
    // this node as the publisher, and this node is alive whether or not the
    // field node is, so the far side's quality is relayed as a topic of its own.
    udpSide.addInputTopic("analogQuality", TOPIC_QUALITY, &analogQuality);

    // Downstream: written from the UDP side, subscribed to from the serial side.
    udpSide.addTopic("command", TOPIC_COMMAND, &command);
    serialSide.addTopic("command", TOPIC_COMMAND, &command);

    // A register crosses the same way, for a stock Modbus tool on either side.
    serialSide.addHreg(200, &sharedRegister);
    udpSide.addHreg(200, &sharedRegister);

    // Placed when the field node joins, and again if it leaves and comes back.
    serialSide.onMemberJoined([&serialSide, fieldNodeID](uint8_t joined) {
        if (joined == fieldNodeID) {
            serialSide.subscribe(fieldNodeID, "analogValue", 0.1f, 10000);
        }
    });

    serialSide.connect();
    udpSide.connect();

    std::printf("gateway %u: serial %s at %u baud, UDP %s:%u, field node %u\n",
                nodeID, serialConfig.device, serialConfig.baudRate,
                udpConfig.groupAddress, udpConfig.port, fieldNodeID);

    bb_time_t lastReport = bb_millis();
    while (g_running) {
        // Both segments are driven from one loop. Neither blocks the other.
        serialSide.loop();
        udpSide.loop();

        // The far side's quality, republished so the UDP side can tell a live
        // value from a stale one.
        analogQuality = (uint8_t)serialSide.topicQuality(fieldNodeID, "analogValue");

        if (bb_elapsed(lastReport, bb_millis()) >= 5000) {
            lastReport = bb_millis();
            std::printf("gateway %u  serial=%u members  udp=%u members  "
                        "value=%g [%s]  command=%d\n",
                        nodeID, serialSide.getMemberCount(), udpSide.getMemberCount(),
                        (double)analogValue,
                        bb_qualityName((BeeBusQuality)analogQuality), command);
            std::fflush(stdout);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    serialSide.end();
    udpSide.end();
    std::printf("gateway %u stopped\n", nodeID);
    return 0;
}
