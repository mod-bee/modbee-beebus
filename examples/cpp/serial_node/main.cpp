// BeeBus node over a serial port, for Linux, macOS and Windows.
//
// Usage:
//   beebus_serial_node <device> <node-id> [baud]
//
// Two nodes with no hardware, using a virtual serial pair:
//   socat -d -d pty,raw,echo=0 pty,raw,echo=0
//   ./beebus_serial_node /dev/pts/3 1
//   ./beebus_serial_node /dev/pts/4 2
//
// A PTY pair is point to point, so it carries exactly two nodes. For larger
// rings use the UDP example, where multicast gives every node the same view.

#include <BeeBusGlobal.h>
#include <transport/BeeBusHostSerialTransport.h>

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
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <device> <node-id> [baud]\n", argv[0]);
        return 2;
    }

    BeeBusSerialConfig config;
    config.device = argv[1];
    config.baudRate = (argc > 3) ? (uint32_t)std::strtoul(argv[3], nullptr, 10) : 115200;

    const uint8_t nodeID = (uint8_t)std::strtoul(argv[2], nullptr, 10);

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    BeeBusHostSerialTransport transport(config);
    // Protocol settings belong to the instance, so several nodes can run in one
    // process with different configuration. Distinct from the transport config
    // above, which describes the link rather than the protocol.
    BeeBusConfig protocolConfig;
    protocolConfig.enableFailSafe = true;

    BeeBusAPI beebus;

    if (!beebus.begin(&transport, nodeID, protocolConfig)) {
        std::fprintf(stderr, "failed to open %s: %s\n",
                     config.device, transport.lastError());
        return 1;
    }

    // Expose two registers so other nodes have something to read.
    int16_t sensorValue = (int16_t)(1000 + nodeID);
    bool alarmActive = false;
    beebus.addHreg(100, &sensorValue);
    beebus.addCoil(25, &alarmActive);

    // begin() only prepares the stack. connect() starts the join sequence.
    beebus.connect();

    std::printf("node %u running on %s at %u baud\n",
                nodeID, config.device, config.baudRate);

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
