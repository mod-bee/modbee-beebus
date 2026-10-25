/**
 * A gateway between a BeeBus RS485 segment and an MQTT broker, on an ESP32.
 *
 * BeeBus side: this node subscribes to topics on the field nodes, on terms
 * stated per topic in the relay table below. MQTT side: each value is published
 * when it changes, alongside its quality.
 *
 *   uplink     beebus/node<N>/<name>            the value
 *              beebus/node<N>/<name>/quality    Good, Uncertain, Bad, Unknown
 *   downlink   beebus/node<N>/<name>/set        written into the field node
 *              beebus/node<N>/<name>/result     what happened to that write
 *   status     beebus/gateway/status            online, or offline by last will
 *
 * Values are published from onTopicChanged rather than on a timer. The deadband
 * and maximum interval on the BeeBus subscription have already decided what
 * deserves to travel; re-sampling on a timer would discard that.
 *
 * Quality is published because a number on a broker has no age. A retained
 * value that stopped updating an hour ago looks identical to a fresh one.
 *
 * Downlink writes are Confirmed and their outcome is published back. MQTT's QoS
 * ends at this gateway and says nothing about whether the value reached the
 * field node.
 *
 * HARDWARE
 * - RS485 transceiver on Serial1 (RX 16, TX 17), at 115200 baud
 * - WiFi for the broker
 *
 * LIBRARY
 * - PubSubClient by Nick O'Leary
 *
 * Numeric topics are relayed from a table of floats; text topics are relayed
 * from a second table of character buffers, because a binding states the type
 * and the type must match the far end.
 */

#include <BeeBusGlobal.h>

#include <WiFi.h>
#include <PubSubClient.h>

#define NODE_ID   90

const char* WIFI_SSID     = "your-ssid";
const char* WIFI_PASSWORD = "your-password";

const char*    MQTT_HOST      = "192.168.1.10";
const uint16_t MQTT_PORT      = 1883;
const char*    MQTT_CLIENT_ID = "beebus-gateway";
const char*    MQTT_USER      = nullptr;
const char*    MQTT_PASSWORD  = nullptr;

// Every topic this gateway publishes sits under this prefix.
const char* MQTT_BASE   = "beebus/";
const char* MQTT_STATUS = "beebus/gateway/status";

const int      RS485_RX_PIN = 16;
const int      RS485_TX_PIN = 17;
const uint32_t BEEBUS_BAUD  = 115200;

const uint32_t MQTT_RETRY_MS   = 5000;
const uint32_t QUALITY_POLL_MS = 1000;

WiFiClient net;
PubSubClient mqtt(net);
BeeBusAPI beebus;

// --- What crosses ---
//
// One entry per value. The variable lives in the entry, so the table is the
// only place a topic is named.
struct Relay {
    uint8_t     nodeID;
    const char* name;
    uint16_t    topicID;
    float       deadband;        // In the value's own units
    uint32_t    maxIntervalMs;   // Report at least this often
    bool        writable;        // Accept a downlink write for it
    float       value;           // Bound to the BeeBus topic
    uint8_t     lastQuality;     // Last quality published
};

Relay relays[] = {
    //  node  name             id  deadband  interval  writable
    {   1,   "analogValue",     1,    0.1f,    60000,   false,  0.0f, 0xFF },
    {   1,   "setpoint",       10,    0.0f,    60000,   true,   0.0f, 0xFF },
    {   2,   "analogValue",     1,    0.1f,    60000,   false,  0.0f, 0xFF },
    {   2,   "setpoint",       10,    0.0f,    60000,   true,   0.0f, 0xFF },
};

const size_t RELAY_COUNT = sizeof(relays) / sizeof(relays[0]);

// Text topics. A fault message is the clearest thing a broker can be given, and
// the one a register map cannot carry without agreeing a length in advance.
// A text topic binds a buffer and its capacity; text arriving longer than the
// buffer is cut to fit and always terminated.
struct TextRelay {
    uint8_t     nodeID;
    const char* name;
    uint16_t    topicID;
    uint32_t    maxIntervalMs;   // No deadband: any change is a new message
    char        value[64];       // Bound to the BeeBus topic
};

TextRelay textRelays[] = {
    //  node  name       id   interval
    {   1,   "fault",   20,   300000,   "" },
    {   2,   "fault",   20,   300000,   "" },
};

const size_t TEXT_RELAY_COUNT = sizeof(textRelays) / sizeof(textRelays[0]);

// --- Helpers ---

int relayByTopic(uint8_t nodeID, uint16_t topicID) {
    for (size_t i = 0; i < RELAY_COUNT; i++) {
        if (relays[i].nodeID == nodeID && relays[i].topicID == topicID) return (int)i;
    }
    return -1;
}

int relayByName(uint8_t nodeID, const char* name) {
    for (size_t i = 0; i < RELAY_COUNT; i++) {
        if (relays[i].nodeID == nodeID && strcmp(relays[i].name, name) == 0) return (int)i;
    }
    return -1;
}

int textRelayByTopic(uint8_t nodeID, uint16_t topicID) {
    for (size_t i = 0; i < TEXT_RELAY_COUNT; i++) {
        if (textRelays[i].nodeID == nodeID && textRelays[i].topicID == topicID) {
            return (int)i;
        }
    }
    return -1;
}

// Builds "beebus/node<N>/<name>" with an optional suffix such as "/quality".
void mqttTopicPath(uint8_t nodeID, const char* name, const char* suffix,
                   char* out, size_t size) {
    snprintf(out, size, "%snode%u/%s%s", MQTT_BASE, (unsigned)nodeID, name,
             suffix ? suffix : "");
}

void mqttTopicFor(const Relay& relay, const char* suffix, char* out, size_t size) {
    mqttTopicPath(relay.nodeID, relay.name, suffix, out, size);
}

void publishValue(const Relay& relay) {
    char topic[96];
    char payload[24];

    mqttTopicFor(relay, nullptr, topic, sizeof(topic));
    snprintf(payload, sizeof(payload), "%.3f", (double)relay.value);
    mqtt.publish(topic, payload, true);
}

void publishQuality(Relay& relay) {
    const BeeBusQuality quality = beebus.topicQuality(relay.nodeID, relay.name);
    relay.lastQuality = (uint8_t)quality;

    char topic[96];
    mqttTopicFor(relay, "/quality", topic, sizeof(topic));
    mqtt.publish(topic, bb_qualityName(quality), true);
}

void publishText(const TextRelay& relay, const char* text) {
    char topic[96];
    mqttTopicPath(relay.nodeID, relay.name, nullptr, topic, sizeof(topic));
    mqtt.publish(topic, text, true);

    mqttTopicPath(relay.nodeID, relay.name, "/quality", topic, sizeof(topic));
    mqtt.publish(topic,
                 bb_qualityName(beebus.topicQuality(relay.nodeID, relay.topicID)), true);
}

// --- BeeBus to MQTT ---

// Text has its own handler because an eight byte scalar cannot carry it, and
// because a store-and-forward flush raises this for every buffered entry while
// the bound variable only ever holds the newest. Reading the variable here
// would publish the newest fault once per entry and lose the history.
void onTopicStringChanged(uint8_t nodeID, uint16_t topicID, const char* text,
                          uint16_t length) {
    (void)length;

    const int index = textRelayByTopic(nodeID, topicID);
    if (index < 0 || !mqtt.connected()) return;

    publishText(textRelays[index], text);
}

void onTopicChanged(uint8_t nodeID, uint16_t topicID, const BeeBusScalar& value) {
    (void)value;   // The bound variable already holds it

    const int index = relayByTopic(nodeID, topicID);
    if (index < 0 || !mqtt.connected()) return;

    publishValue(relays[index]);
    publishQuality(relays[index]);
}

// A publish that never arrived is worth knowing about on the broker side too.
void onTopicGap(uint8_t nodeID, uint16_t topicID, uint8_t missed) {
    const int index = relayByTopic(nodeID, topicID);
    if (index < 0 || !mqtt.connected()) return;

    char topic[96];
    char payload[16];
    mqttTopicFor(relays[index], "/missed", topic, sizeof(topic));
    snprintf(payload, sizeof(payload), "%u", (unsigned)missed);
    mqtt.publish(topic, payload, false);
}

// --- MQTT to BeeBus ---

// Splits "beebus/node<N>/<name>/set" into its node and name.
bool parseSetTopic(const char* topic, uint8_t& nodeID, char* name, size_t nameSize) {
    const size_t baseLen = strlen(MQTT_BASE);
    if (strncmp(topic, MQTT_BASE, baseLen) != 0) return false;

    const char* cursor = topic + baseLen;
    if (strncmp(cursor, "node", 4) != 0) return false;
    cursor += 4;

    char* end = nullptr;
    const long parsed = strtol(cursor, &end, 10);
    if (end == cursor || *end != '/' || parsed < BEEBUS_NODE_ID_MIN ||
        parsed > BEEBUS_NODE_ID_MAX) {
        return false;
    }
    nodeID = (uint8_t)parsed;
    cursor = end + 1;

    const char* tail = strstr(cursor, "/set");
    if (!tail || tail[4] != '\0') return false;

    const size_t length = (size_t)(tail - cursor);
    if (length == 0 || length >= nameSize) return false;

    memcpy(name, cursor, length);
    name[length] = '\0';
    return true;
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    uint8_t nodeID = 0;
    char name[BEEBUS_TOPIC_NAME_MAX];
    if (!parseSetTopic(topic, nodeID, name, sizeof(name))) return;

    const int index = relayByName(nodeID, name);
    if (index < 0 || !relays[index].writable) return;

    char text[24];
    const unsigned int copied = length < sizeof(text) - 1 ? length : sizeof(text) - 1;
    memcpy(text, payload, copied);
    text[copied] = '\0';

    // Confirmed, so the outcome can be reported. MQTT's own delivery guarantee
    // stops at this gateway.
    beebus.writeTopic(nodeID, name, (float)atof(text), BeeBusDelivery::Confirmed);
}

void onWriteComplete(uint8_t nodeID, uint16_t topicID, BeeBusResult result) {
    const int index = relayByTopic(nodeID, topicID);
    if (index < 0 || !mqtt.connected()) return;

    char topic[96];
    mqttTopicFor(relays[index], "/result", topic, sizeof(topic));
    mqtt.publish(topic, bb_resultName(result), false);
}

// --- Connection management ---

void subscribeToNode(uint8_t nodeID) {
    // Placed when a node joins, and again if it leaves and comes back.
    for (size_t i = 0; i < RELAY_COUNT; i++) {
        if (relays[i].nodeID != nodeID) continue;
        beebus.subscribe(nodeID, relays[i].name, relays[i].deadband,
                         relays[i].maxIntervalMs);
    }

    // A deadband means nothing for text, so any change is reported. The
    // interval is what makes a silent fault topic distinguishable from a
    // healthy one.
    for (size_t i = 0; i < TEXT_RELAY_COUNT; i++) {
        if (textRelays[i].nodeID != nodeID) continue;
        beebus.subscribe(nodeID, textRelays[i].name, 0.0f, textRelays[i].maxIntervalMs);
    }
}

void connectMqtt() {
    // The broker being unreachable must not stop the BeeBus segment, so this is
    // attempted at intervals rather than waited on. PubSubClient's connect()
    // does block for the length of the TCP attempt.
    static uint32_t lastAttempt = 0;
    if (millis() - lastAttempt < MQTT_RETRY_MS) return;
    lastAttempt = millis();

    // Last will: the broker says so on our behalf if this gateway drops off.
    if (!mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD,
                      MQTT_STATUS, 0, true, "offline")) {
        return;
    }

    mqtt.publish(MQTT_STATUS, "online", true);
    mqtt.subscribe("beebus/+/+/set");

    // Republish everything. onTopicChanged will not fire again for values that
    // have not moved, so a reconnect would otherwise leave the broker empty
    // until each value next changes.
    for (size_t i = 0; i < RELAY_COUNT; i++) {
        publishValue(relays[i]);
        publishQuality(relays[i]);
    }
    for (size_t i = 0; i < TEXT_RELAY_COUNT; i++) {
        publishText(textRelays[i], beebus.topicText(textRelays[i].nodeID,
                                                    textRelays[i].topicID));
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(BEEBUS_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
    }

    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(onMqttMessage);

    if (!beebus.begin(&Serial1, NODE_ID, BeeBusConfig(), BEEBUS_BAUD)) {
        Serial.println(F("BeeBus initialisation failed"));
        while (1) delay(1000);
    }

    // A value arriving for a topic nothing is bound to is dropped, so every
    // relayed topic is bound before anything is asked of it.
    for (size_t i = 0; i < RELAY_COUNT; i++) {
        beebus.registerTopic(relays[i].nodeID, relays[i].name, relays[i].topicID,
                             &relays[i].value);
    }
    for (size_t i = 0; i < TEXT_RELAY_COUNT; i++) {
        beebus.registerTopic(textRelays[i].nodeID, textRelays[i].name,
                             textRelays[i].topicID, textRelays[i].value,
                             sizeof(textRelays[i].value));
    }

    beebus.onMemberJoined(subscribeToNode);
    beebus.onTopicChanged(onTopicChanged);
    beebus.onTopicStringChanged(onTopicStringChanged);
    beebus.onTopicGap(onTopicGap);
    beebus.onWriteComplete(onWriteComplete);

    beebus.connect();
}

void loop() {
    beebus.loop();

    if (mqtt.connected()) {
        mqtt.loop();
    } else {
        connectMqtt();
    }

    // Quality changes without the value changing: a publisher that goes silent
    // never raises onTopicChanged again, which is exactly when the broker most
    // needs to be told.
    static uint32_t lastPoll = 0;
    if (millis() - lastPoll < QUALITY_POLL_MS) return;
    lastPoll = millis();

    if (!mqtt.connected()) return;

    for (size_t i = 0; i < RELAY_COUNT; i++) {
        if ((uint8_t)beebus.topicQuality(relays[i].nodeID, relays[i].name) !=
            relays[i].lastQuality) {
            publishQuality(relays[i]);
        }
    }
}
