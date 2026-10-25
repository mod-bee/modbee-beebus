// Topics between nodes over UDP multicast, for Linux, macOS and Windows.
//
// Usage:
//   beebus_topic_node <node-id> <peer-id> [group] [port]
//
// Two nodes on one machine:
//   ./beebus_topic_node 1 2
//   ./beebus_topic_node 2 1
//
// Each node binds four topics of its own and the peer's three read-only ones,
// then exercises every topic operation against the peer: subscribe, publish,
// readTopic, writeTopic and fetchTopics.
//
// A topic is named where it is bound and named in every call afterwards; the ID
// is what travels on the wire. The type comes from the bound variable and is
// checked at the far end.

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
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <node-id> <peer-id> [group] [port]\n", argv[0]);
        return 2;
    }

    const uint8_t nodeID = (uint8_t)std::strtoul(argv[1], nullptr, 10);
    const uint8_t peerID = (uint8_t)std::strtoul(argv[2], nullptr, 10);

    BeeBusUdpConfig config;
    if (argc > 3) config.groupAddress = argv[3];
    if (argc > 4) config.port = (uint16_t)std::strtoul(argv[4], nullptr, 10);

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    BeeBusHostUdpTransport transport(config);
    BeeBusAPI beebus;

    if (!beebus.begin(&transport, nodeID)) {
        std::fprintf(stderr, "node %u: cannot open %s:%u: %s\n", nodeID,
                     config.groupAddress, config.port, transport.lastError());
        return 1;
    }

    // --- Topics this node owns ---
    float    analogValue = (float)nodeID;
    uint32_t counter     = 0;
    bool     flag        = false;
    int16_t  command     = 0;

    // addInputTopic is read-only to peers; addTopic may also be written.
    beebus.addInputTopic("analogValue", 1, &analogValue);
    beebus.addInputTopic("counter",     2, &counter);
    beebus.addInputTopic("flag",        3, &flag);
    beebus.addTopic("command",         10, &command);

    // --- The peer's topics, bound to variables of ours ---
    float    peerAnalogValue = 0.0f;
    uint32_t peerCounter     = 0;
    bool     peerFlag        = false;

    beebus.registerTopic(peerID, "analogValue", 1, &peerAnalogValue);
    beebus.registerTopic(peerID, "counter",     2, &peerCounter);
    beebus.registerTopic(peerID, "flag",        3, &peerFlag);

    beebus.onTopicWritten([nodeID](uint8_t from, uint16_t topicID, const BeeBusScalar& value) {
        std::printf("node %u: node %u wrote topic %u = %g\n",
                    nodeID, from, topicID, (double)bb_scalarToFloat(value));
    });

    // Every publish carries a sequence number for its destination, so a loss is
    // reported rather than showing as a value that stopped changing.
    beebus.onTopicGap([nodeID](uint8_t from, uint16_t topicID, uint8_t missed) {
        std::printf("node %u: missed %u publishes of topic %u from node %u\n",
                    nodeID, missed, topicID, from);
    });

    beebus.onTopicList([nodeID](uint8_t from, const BeeBusTopicEntry& entry) {
        std::printf("node %u: node %u topic %u  %-12s %s\n",
                    nodeID, from, entry.topicID, entry.name,
                    bb_topicTypeName(entry.type));
    });

    // Subscriptions are placed when the peer joins, and again if it leaves and
    // comes back: a node that restarts has lost its subscriber list.
    beebus.onMemberJoined([&beebus, peerID](uint8_t joined) {
        if (joined != peerID) return;

        // Deadband in the value's own units, then the longest gap this node
        // would still accept the value after. An interval of 0 promises nothing.
        beebus.subscribe(peerID, "analogValue", 0.5f, 2000);
        beebus.subscribe(peerID, "counter",    10.0f, 30000);
        beebus.subscribe(peerID, "flag",        0.0f, 30000);

        // Walks the peer's whole list, a page per token.
        beebus.fetchTopics(peerID);
    });

    beebus.connect();
    std::printf("node %u running on %s:%u, peer %u\n",
                nodeID, config.groupAddress, config.port, peerID);

    bb_time_t lastTick = bb_millis();
    while (g_running) {
        beebus.loop();

        if (bb_elapsed(lastTick, bb_millis()) >= 1000) {
            lastTick = bb_millis();

            // The subscribed topics need no publish() call: the peer stated its
            // terms and this node honours them on its own.
            analogValue += 0.25f;
            counter++;

            // An event, sent the moment it happens.
            if ((counter % 10) == 0) {
                flag = !flag;
                beebus.publishAll("flag");
            }

            if ((counter % 5) == 0) {
                beebus.readTopic(peerID, "counter");
                beebus.writeTopic(peerID, "command", (int16_t)counter,
                                  BeeBusDelivery::Confirmed);
            }

            std::printf("node %u  peer analogValue=%g [%s]  counter=%u  silence=%u ms\n",
                        nodeID, (double)peerAnalogValue,
                        bb_qualityName(beebus.topicQuality(peerID, "analogValue")),
                        peerCounter, beebus.silenceFor(peerID));
            std::fflush(stdout);
        }

        // Yield briefly so polling does not saturate a core.
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    beebus.end();
    std::printf("node %u stopped\n", nodeID);
    return 0;
}
