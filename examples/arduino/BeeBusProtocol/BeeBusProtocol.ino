/**
 * A BeeBus node exposing a full Modbus-style register map, and exchanging
 * values with a second node.
 *
 * This node binds its own variables as coils, holding registers, discrete
 * inputs and input registers, so any peer (or a Modbus gateway on the ring)
 * can read and write them. Once a second it also writes some of its values
 * to node 2 and reads node 2's inputs back.
 *
 * Inputs are simulated so the sketch runs on a bare board; replace the
 * simulated values with real reads for your hardware.
 *
 * HARDWARE
 * - RS485 transceiver on Serial2, at 115200 baud
 */

#include <BeeBusGlobal.h>

#define NODE_ID         1
#define REMOTE_NODE_ID  2   // The node this one exchanges values with
#define BEEBUS_BAUD     115200

BeeBusAPI beebus;

// --- Local variables, bound to this node's register map ---

// Digital outputs (coils 0-7), writable by peers
bool relay1 = false;
bool relay2 = false;
bool led1 = false;
bool led2 = false;
bool valve1 = false;
bool valve2 = false;
bool pump1 = false;
bool pump2 = false;

// Analog outputs (holding registers 0-3), writable by peers
int16_t analogOut1 = 0;     // 0-10V output (0-10000 mV)
int16_t analogOut2 = 0;     // 4-20mA output (4000-20000 uA)
int16_t setpoint1 = 0;      // Temperature setpoint
int16_t setpoint2 = 0;      // Pressure setpoint

// Digital inputs (discrete inputs 0-7), read-only to peers
bool button1 = false;
bool button2 = false;
bool sensor1 = false;
bool sensor2 = false;
bool limit1 = false;
bool limit2 = false;
bool alarm1 = false;
bool alarm2 = false;

// Analog inputs (input registers 0-3), read-only to peers
int16_t temperature = 0;    // Temperature sensor (degC * 10)
int16_t pressure = 0;       // Pressure sensor (kPa)
int16_t flow = 0;           // Flow sensor (L/min * 10)
int16_t level = 0;          // Level sensor (% * 10)

// --- Buffers for operations on the remote node ---
bool remoteOutputs[8];      // Written to the remote node's coils
int16_t remoteAnalog[4];    // Written to the remote node's holding registers
bool remoteInputs[8];       // Read from the remote node's discrete inputs
int16_t remoteSensors[4];   // Read from the remote node's input registers

// --- Callbacks ---

void onBeebusError(BeeBusError error, const char* message) {
    Serial.print("BeeBus Error: ");
    Serial.print((int)error);
    Serial.print(" - ");
    Serial.println(message);
}

// Debug output is compiled out on boards unless the library is built with
// BEEBUS_ENABLE_DEBUG defined; without it this handler is never called.
void onBeebusDebug(const char* category, const char* message) {
    Serial.print("[");
    Serial.print(category);
    Serial.print("] ");
    Serial.println(message);
}

void setup() {
    Serial.begin(115200);
    Serial.println("BeeBus Protocol Example Starting...");

    // Nothing here has to be set for the network to work. Every timing figure
    // is measured, so the same sketch suits this wired segment and the same
    // segment over a radio, and adding a node later changes nothing.
    //
    // Failsafe is a policy choice rather than a timing one: it clears registers
    // belonging to a node that is lost, so a stale command cannot hold a
    // physical output energised.
    BeeBusConfig config;
    config.enableFailSafe = true;

    // The baud rate is passed too: it is the one property of the line a node
    // cannot measure for itself.
    Serial2.begin(BEEBUS_BAUD);
    if (!beebus.begin(&Serial2, NODE_ID, config, BEEBUS_BAUD)) {
        Serial.println("Failed to initialise BeeBus!");
        while (1) delay(1000);
    }

    beebus.onError(onBeebusError);
    beebus.onDebug(onBeebusDebug);

    // --- Bind local variables to the register map ---

    beebus.addCoil(0, &relay1);
    beebus.addCoil(1, &relay2);
    beebus.addCoil(2, &led1);
    beebus.addCoil(3, &led2);
    beebus.addCoil(4, &valve1);
    beebus.addCoil(5, &valve2);
    beebus.addCoil(6, &pump1);
    beebus.addCoil(7, &pump2);

    beebus.addHreg(0, &analogOut1);
    beebus.addHreg(1, &analogOut2);
    beebus.addHreg(2, &setpoint1);
    beebus.addHreg(3, &setpoint2);

    beebus.addIsts(0, &button1);
    beebus.addIsts(1, &button2);
    beebus.addIsts(2, &sensor1);
    beebus.addIsts(3, &sensor2);
    beebus.addIsts(4, &limit1);
    beebus.addIsts(5, &limit2);
    beebus.addIsts(6, &alarm1);
    beebus.addIsts(7, &alarm2);

    beebus.addIreg(0, &temperature);
    beebus.addIreg(1, &pressure);
    beebus.addIreg(2, &flow);
    beebus.addIreg(3, &level);

    // begin() only prepares the stack. connect() starts the join sequence.
    beebus.connect();

    Serial.print("BeeBus node ");
    Serial.print(NODE_ID);
    Serial.println(" started, waiting for the network...");
}

void loop() {
    // Call every pass, as often as possible: the protocol does all its work,
    // including answering peers, from here.
    beebus.loop();

    // --- Local inputs (simulated) ---
    static unsigned long lastInputRead = 0;
    if (millis() - lastInputRead > 100) {
        // Replace with real reads, e.g. button1 = digitalRead(BUTTON1_PIN);
        button1 = (millis() / 1000) % 2;
        sensor1 = (millis() / 2000) % 2;
        alarm1 = (millis() / 5000) % 2;

        lastInputRead = millis();
    }

    static unsigned long lastAnalogRead = 0;
    if (millis() - lastAnalogRead > 500) {
        // Replace with real reads from your ADC.
        temperature = 250 + (sin(millis() / 10000.0) * 50);  // 20-30 degC
        pressure = 1000 + (cos(millis() / 8000.0) * 200);    // 800-1200 kPa
        flow = 150 + (sin(millis() / 6000.0) * 50);          // 10-20 L/min
        level = 500 + (cos(millis() / 12000.0) * 300);       // 20-80%

        lastAnalogRead = millis();
    }

    // --- Local outputs ---
    // The coil and holding register variables above change when a peer writes
    // them. Apply them to your hardware here, e.g.
    // digitalWrite(RELAY1_PIN, relay1);

    // --- Exchange with the remote node ---
    if (beebus.isConnected()) {
        static unsigned long lastRemoteComm = 0;

        if (millis() - lastRemoteComm > 1000) {
            remoteOutputs[0] = button1;
            remoteOutputs[1] = sensor1;
            remoteAnalog[0] = temperature;
            remoteAnalog[1] = pressure;

            // Operations are queued and sent on this node's next turn with the
            // token. A BeeBusResult that bb_accepted() rejects means the
            // operation was refused outright (unknown node, queue full, ...).
            if (bb_accepted(beebus.writeCoil(REMOTE_NODE_ID, 0, remoteOutputs))) {
                Serial.println("Queued digital outputs for the remote node");
            }
            if (bb_accepted(beebus.writeHreg(REMOTE_NODE_ID, 0, remoteAnalog))) {
                Serial.println("Queued analog values for the remote node");
            }

            // A read fills its buffer when the answer arrives on a later
            // loop(), so the values printed here are from the previous read.
            // onReadComplete() reports exactly when each one lands.
            if (bb_accepted(beebus.readIsts(REMOTE_NODE_ID, 0, remoteInputs))) {
                Serial.print("Remote digital inputs: ");
                for (int i = 0; i < 8; i++) {
                    Serial.print(remoteInputs[i] ? "1" : "0");
                    Serial.print(" ");
                }
                Serial.println();
            }

            if (bb_accepted(beebus.readIreg(REMOTE_NODE_ID, 0, remoteSensors))) {
                Serial.print("Remote analog inputs: ");
                for (int i = 0; i < 4; i++) {
                    Serial.print(remoteSensors[i]);
                    Serial.print(" ");
                }
                Serial.println();
            }

            lastRemoteComm = millis();
        }
    }

    // --- Status, every 5 seconds ---
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 5000) {
        Serial.println("=== BeeBus Status ===");
        Serial.print("Node ID: ");
        Serial.println(beebus.getNodeID());
        Serial.print("Connected: ");
        Serial.println(beebus.isConnected() ? "YES" : "NO");
        Serial.print("Pending Operations: ");
        Serial.println(beebus.getPendingOpCount());

        Serial.println("--- Local Values ---");
        Serial.print("Digital Outputs: ");
        Serial.print(relay1); Serial.print(" ");
        Serial.print(relay2); Serial.print(" ");
        Serial.print(led1); Serial.print(" ");
        Serial.print(led2); Serial.println();

        Serial.print("Analog Outputs: ");
        Serial.print(analogOut1); Serial.print(" ");
        Serial.print(analogOut2); Serial.print(" ");
        Serial.print(setpoint1); Serial.print(" ");
        Serial.print(setpoint2); Serial.println();

        Serial.print("Digital Inputs: ");
        Serial.print(button1); Serial.print(" ");
        Serial.print(sensor1); Serial.print(" ");
        Serial.print(alarm1); Serial.println();

        Serial.print("Analog Inputs: ");
        Serial.print(temperature); Serial.print(" ");
        Serial.print(pressure); Serial.print(" ");
        Serial.print(flow); Serial.print(" ");
        Serial.print(level); Serial.println();

        Serial.println("=====================");

        lastStatus = millis();
    }
}
