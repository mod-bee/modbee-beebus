// BeeBus node over UDP multicast, for Linux, macOS and Windows.
//
// Usage:
//   beebus_udp_node <node-id> [group] [port]
//
// Several nodes on one machine, no hardware and no virtual ports:
//   ./beebus_udp_node 1
//   ./beebus_udp_node 2
//   ./beebus_udp_node 3
//
// The same binary works unchanged across Docker containers on a shared bridge
// network, which is how a larger simulated ring is built.

#include <BeeBusGlobal.h>
#include <transport/BeeBusHostUdpTransport.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {

volatile std::sig_atomic_t g_running = 1;

void onSignal(int) { g_running = 0; }

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <node-id> [group] [port]\n", argv[0]);
        return 2;
    }

    const uint8_t nodeID = (uint8_t)std::strtoul(argv[1], nullptr, 10);

    BeeBusUdpConfig config;
    if (argc > 2) config.groupAddress = argv[2];
    if (argc > 3) config.port = (uint16_t)std::strtoul(argv[3], nullptr, 10);

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    BeeBusHostUdpTransport transport(config);
    // Protocol settings belong to the instance, so several nodes can run in one
    // process with different configuration. Distinct from the transport config
    // above, which describes the link rather than the protocol.
    BeeBusConfig protocolConfig;
    protocolConfig.enableFailSafe = true;

    BeeBusAPI beebus;

    if (!beebus.begin(&transport, nodeID, protocolConfig)) {
        std::fprintf(stderr, "failed to open group %s:%u: %s\n",
                     config.groupAddress, config.port, transport.lastError());
        return 1;
    }

    // Expose two registers so other nodes have something to read.
    int16_t sensorValue = (int16_t)(1000 + nodeID);
    bool alarmActive = false;
    beebus.addHreg(100, &sensorValue);
    beebus.addCoil(25, &alarmActive);

    // begin() only prepares the stack. connect() starts the join sequence.
    beebus.connect();

    std::printf("node %u running on %s:%u\n",
                nodeID, config.groupAddress, config.port);

    bb_time_t lastReport = bb_millis();
    while (g_running) {
        beebus.loop();

        if (bb_elapsed(lastReport, bb_millis()) >= 1000) {
            lastReport = bb_millis();
            sensorValue++;
            alarmActive = !alarmActive;
            std::printf("node %u  connected=%d  hreg100=%d\n",
                        nodeID, (int)beebus.isConnected(), sensorValue);
            std::fflush(stdout);
        }

        // Yield briefly so polling does not saturate a core.
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    beebus.end();
    std::printf("node %u stopped\n", nodeID);
    return 0;
}
