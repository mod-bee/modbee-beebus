/**
 * Topics between two nodes.
 *
 * Node 1 and Node 2 run identical code apart from NODE_ID and PEER_ID. Each
 * binds four topics of its own and the peer's four, then exercises every topic
 * operation against the peer:
 *
 *   subscribe   the peer reports a value on a deadband and a maximum interval
 *   publish     an event is sent the moment it happens
 *   readTopic   a value is asked for on demand
 *   writeTopic  a value is written into the peer's topic
 *   fetchTopics the peer's topic list is fetched by name
 *
 * A topic is named where it is bound and named in every call afterwards; the
 * ID is what travels on the wire. The type comes from the bound variable and is
 * checked at the far end.
 *
 * HARDWARE
 * - RS485 transceiver on Serial1, at 115200 baud
 */

#include <BeeBusGlobal.h>

#define NODE_ID   1
#define PEER_ID   2

BeeBusAPI beebus;

void subscribeToPeer();
void report();

#define BEEBUS_BAUD 115200

// --- Topics this node owns ---
//
// A text topic binds a character buffer and its capacity. An Arduino String
// reaches any call taking text as myString.c_str().
float    analogValue = 0.0f;   // Changes continuously
uint32_t counter     = 0;      // Advances every second
bool     flag        = false;  // Changes rarely
int16_t  command     = 0;      // Written by the peer
char     status[32]  = "";     // Text, up to 31 characters here

// --- The peer's topics, bound to variables of ours ---
float    peerAnalogValue = 0.0f;
uint32_t peerCounter     = 0;
bool     peerFlag        = false;
char     peerStatus[32]  = "";

void setup() {
    Serial.begin(115200);
    Serial1.begin(BEEBUS_BAUD);

    // The baud rate is passed too: it is the one property of the line a node
    // cannot measure for itself.
    beebus.begin(&Serial1, NODE_ID, BeeBusConfig(), BEEBUS_BAUD);

    // addInputTopic is read-only to peers; addTopic may also be written.
    beebus.addInputTopic("analogValue", 1, &analogValue);
    beebus.addInputTopic("counter",     2, &counter);
    beebus.addInputTopic("flag",        3, &flag);
    beebus.addInputTopic("status",      4, status, sizeof(status));
    beebus.addTopic("command",         10, &command);

    // Values arriving for a topic nothing is bound to are dropped, so the
    // peer's topics are bound before anything is asked of them.
    beebus.registerTopic(PEER_ID, "analogValue", 1, &peerAnalogValue);
    beebus.registerTopic(PEER_ID, "counter",     2, &peerCounter);
    beebus.registerTopic(PEER_ID, "flag",        3, &peerFlag);
    beebus.registerTopic(PEER_ID, "status",      4, peerStatus, sizeof(peerStatus));

    // Text has its own handler: an eight byte scalar cannot carry it, and a
    // buffered flush raises this for every entry while the variable holds only
    // the newest.
    beebus.onTopicStringChanged([](uint8_t node, uint16_t topicID,
                                   const char* text, uint16_t) {
        Serial.print(F("node "));
        Serial.print(node);
        Serial.print(F(" topic "));
        Serial.print(topicID);
        Serial.print(F(" = "));
        Serial.println(text);
    });

    beebus.onTopicChanged([](uint8_t node, uint16_t topicID, const BeeBusScalar&) {
        Serial.print(F("published by node "));
        Serial.print(node);
        Serial.print(F(" topic "));
        Serial.println(topicID);
    });

    beebus.onTopicWritten([](uint8_t node, uint16_t topicID, const BeeBusScalar&) {
        Serial.print(F("written by node "));
        Serial.print(node);
        Serial.print(F(" topic "));
        Serial.println(topicID);
    });

    // Every publish carries a sequence number for its destination, so a loss is
    // reported rather than showing as a value that stopped changing.
    beebus.onTopicGap([](uint8_t node, uint16_t topicID, uint8_t missed) {
        Serial.print(F("missed "));
        Serial.print(missed);
        Serial.print(F(" publishes of topic "));
        Serial.print(topicID);
        Serial.print(F(" from node "));
        Serial.println(node);
    });

    // One entry per topic the peer publishes, name and type included.
    beebus.onTopicList([](uint8_t node, const BeeBusTopicEntry& entry) {
        Serial.print(F("node "));
        Serial.print(node);
        Serial.print(F(" topic "));
        Serial.print(entry.topicID);
        Serial.print(F("  "));
        Serial.print(entry.name);
        Serial.print(F("  "));
        Serial.println(bb_topicTypeName(entry.type));
    });

    // Subscriptions are placed when the peer joins, and again if it leaves and
    // comes back: a node that restarts has lost its subscriber list.
    beebus.onMemberJoined([](uint8_t node) {
        if (node == PEER_ID) subscribeToPeer();
    });

    beebus.connect();
}

void subscribeToPeer() {
    // Deadband in the value's own units, then the longest gap this node would
    // still accept the value after. An interval of 0 promises nothing.
    beebus.subscribe(PEER_ID, "analogValue", 0.5f, 2000);
    beebus.subscribe(PEER_ID, "counter",    10.0f, 30000);
    beebus.subscribe(PEER_ID, "flag",        0.0f, 30000);

    // A deadband means nothing for text, so any change is reported. The
    // interval still is what makes silence meaningful.
    beebus.subscribe(PEER_ID, "status",      0.0f, 30000);

    // Walks the peer's whole list, a page per token, through onTopicList above.
    beebus.fetchTopics(PEER_ID);
}

void report() {
    // Whether the value can be acted on, judged against the interval the
    // subscription asked for.
    Serial.print(F("analogValue "));
    Serial.print(peerAnalogValue);
    Serial.print(F(" ["));
    Serial.print(bb_qualityName(beebus.topicQuality(PEER_ID, "analogValue")));
    Serial.print(F("] counter "));
    Serial.print(peerCounter);
    Serial.print(F(" status "));
    Serial.print(beebus.topicText(PEER_ID, "status"));
    Serial.print(F(" peer last heard "));
    Serial.print(beebus.silenceFor(PEER_ID));
    Serial.println(F(" ms ago"));
}

void loop() {
    beebus.loop();

    static uint32_t lastTick = 0;
    if (millis() - lastTick < 1000) return;
    lastTick = millis();

    // --- This node's own values ---
    //
    // The subscribed topics need no publish() call: the peer stated its terms
    // and this node honours them on its own.
    analogValue += 0.25f;
    counter++;

    // An event, sent the moment it happens rather than waiting for an interval.
    if ((counter % 10) == 0) {
        flag = !flag;

        // publishAll sends one section the whole ring reads, and whoever
        // holds the topic takes it — one turn on the bus however many are
        // listening. It carries no sequence, so it sits alongside the
        // subscribers' own stream rather than interleaving with it.
        beebus.publishAll("flag");

        // Text of whatever length the message needs, up to the buffer's
        // capacity. Nothing at the far end was told how long it would be.
        strncpy(status, flag ? "running" : "stopped on low level", sizeof(status) - 1);
        status[sizeof(status) - 1] = '\0';
    }

    // --- Asked of the peer ---
    if ((counter % 5) == 0) {
        // On demand: the answer lands in peerCounter, and onReadComplete says
        // when, with the topic ID as its address.
        beebus.readTopic(PEER_ID, "counter");

        // Confirmed, so the result is reported rather than assumed.
        beebus.writeTopic(PEER_ID, "command", (int16_t)counter,
                          BeeBusDelivery::Confirmed);
    }

    report();
}
