/**
 * BeeBus over WiFi UDP on an ESP32.
 *
 * Shows that the transport, not the platform, decides the medium. The sketch
 * is identical to a serial node apart from the transport it hands to begin().
 *
 * All nodes must join the same multicast group, which is what gives the token
 * ring the shared-bus behaviour it expects.
 *
 * HARDWARE
 * - ESP32 on WiFi
 * - An analog sensor on A0, published as holding register 100
 */

#include <BeeBusGlobal.h>
#include <transport/BeeBusArduinoUdpTransport.h>

#include <WiFi.h>
#include <WiFiUdp.h>

// =============================================================================
// CONFIGURATION
// =============================================================================
#define NODE_ID 1

const char* WIFI_SSID = "your-ssid";
const char* WIFI_PASSWORD = "your-password";

// =============================================================================
// GLOBALS
// =============================================================================
WiFiUDP udp;
BeeBusUdpConfig udpConfig;
BeeBusArduinoUdpTransport transport(&udp, udpConfig);
BeeBusAPI beebus;

int16_t sensorValue = 0;
bool alarmActive = false;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Join the group before handing the socket to the transport.
    IPAddress group;
    group.fromString(udpConfig.groupAddress);
    if (!udp.beginMulticast(group, udpConfig.port)) {
        Serial.println("Failed to join multicast group");
        while (1) delay(1000);
    }

    // Protocol settings, distinct from udpConfig which describes the link.
    BeeBusConfig protocolConfig;
    protocolConfig.enableFailSafe = true;

    if (!beebus.begin(&transport, NODE_ID, protocolConfig)) {
        Serial.println("BeeBus initialisation failed");
        while (1) delay(1000);
    }

    beebus.addHreg(100, &sensorValue);
    beebus.addCoil(25, &alarmActive);

    // begin() only prepares the stack. connect() starts the join sequence.
    beebus.connect();

    Serial.println("BeeBus UDP node running");
}

void loop() {
    beebus.loop();

    static uint32_t lastSample = 0;
    if (millis() - lastSample >= 100) {
        lastSample = millis();
        sensorValue = analogRead(A0);
        alarmActive = sensorValue > 2048;
    }
}
