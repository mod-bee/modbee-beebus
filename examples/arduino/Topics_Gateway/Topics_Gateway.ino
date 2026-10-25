/**
 * A gateway between an RS485 segment and a WiFi UDP segment, on an ESP32.
 *
 * The two segments are separate BeeBus networks: separate token rings, separate
 * address spaces, separate measured timing. This node is a member of both, so it
 * runs two BeeBusAPI instances, one per transport, and calls loop() on each.
 *
 * A value crosses because the same variable is bound into both maps. Nothing
 * else is forwarded, and neither direction needs forwarding code: the publish
 * scan on the far side sees the variable change and reports it.
 *
 *   upstream    registered and subscribed on the serial side, owned on the UDP
 *               side from the same variable
 *   downstream  owned on the UDP side so it can be written, owned on the serial
 *               side so it can be subscribed to, from the same variable
 *
 * Pairs with Topics_Node1 on the serial segment, whose analogValue it relays.
 * On the UDP segment, any node may subscribe to node 10's analogValue or write
 * its command topic, which nodes on the serial segment may subscribe to.
 *
 * HARDWARE
 * - RS485 transceiver on Serial1 (RX 16, TX 17), at 115200 baud
 * - WiFi for the UDP segment
 *
 * Two instances means two complete nodes' worth of buffers and tables, of the
 * order of 50 KB of RAM together. Comfortable on an ESP32.
 */

#include <BeeBusGlobal.h>
#include <transport/BeeBusArduinoUdpTransport.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#define NODE_ID     10   // The same ID on both segments: separate rings
#define FIELD_NODE  1    // On the serial segment

const char* WIFI_SSID = "your-ssid";
const char* WIFI_PASSWORD = "your-password";

const int      RS485_RX_PIN = 16;
const int      RS485_TX_PIN = 17;
const uint32_t BEEBUS_BAUD  = 115200;

// Topic IDs. Each segment numbers its own, so these need only agree with the
// far node for the one topic registered from it.
const uint16_t TOPIC_VALUE   = 1;
const uint16_t TOPIC_QUALITY = 2;
const uint16_t TOPIC_COMMAND = 10;

WiFiUDP udp;
BeeBusUdpConfig udpConfig;
BeeBusArduinoUdpTransport udpTransport(&udp, udpConfig);

// One instance per segment. The config belongs to the instance, so the two
// sides measure their own timing and never share a timeout.
BeeBusAPI serialSide, udpSide;

// --- The variables that cross ---
float   analogValue    = 0.0f;
uint8_t analogQuality  = (uint8_t)BeeBusQuality::Unknown;
int16_t command        = 0;
int16_t sharedRegister = 0;

void setup() {
    Serial.begin(115200);
    Serial1.begin(BEEBUS_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
    }

    // Join the group before handing the socket to the transport.
    IPAddress group;
    group.fromString(udpConfig.groupAddress);
    if (!udp.beginMulticast(group, udpConfig.port)) {
        Serial.println(F("failed to join multicast group"));
        while (1) delay(1000);
    }

    if (!serialSide.begin(&Serial1, NODE_ID, BeeBusConfig(), BEEBUS_BAUD) ||
        !udpSide.begin(&udpTransport, NODE_ID)) {
        Serial.println(F("BeeBus initialisation failed"));
        while (1) delay(1000);
    }

    // Upstream: the field node's topic lands in analogValue, and the UDP side
    // publishes the same variable to whoever subscribes there.
    serialSide.registerTopic(FIELD_NODE, "analogValue", TOPIC_VALUE, &analogValue);
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
    serialSide.onMemberJoined([](uint8_t joined) {
        if (joined == FIELD_NODE) {
            serialSide.subscribe(FIELD_NODE, "analogValue", 0.1f, 10000);
        }
    });

    serialSide.connect();
    udpSide.connect();
}

void loop() {
    // Both segments are driven from one loop. Neither blocks the other.
    serialSide.loop();
    udpSide.loop();

    // The far side's quality, republished so the UDP side can tell a live value
    // from a stale one.
    analogQuality = (uint8_t)serialSide.topicQuality(FIELD_NODE, "analogValue");

    static uint32_t lastReport = 0;
    if (millis() - lastReport < 5000) return;
    lastReport = millis();

    Serial.print(F("serial members "));
    Serial.print(serialSide.getMemberCount());
    Serial.print(F("  udp members "));
    Serial.print(udpSide.getMemberCount());
    Serial.print(F("  value "));
    Serial.print(analogValue);
    Serial.print(F(" ["));
    Serial.print(bb_qualityName((BeeBusQuality)analogQuality));
    Serial.print(F("]  command "));
    Serial.println(command);
}
