/**
 * A transparent I/O link between two nodes, over registers.
 * Both nodes run identical code, except for the NODE_ID and REMOTE_NODE_ID.
 *
 * Each node reads its own inputs and WRITES them to the other node's outputs.
 * There are no READ commands issued. The BeeBus library automatically handles
 * receiving writes and updating the output variables.
 *
 * HARDWARE SETUP (Node 2):
 * - RS485 transceiver on Serial1, at 115200 baud
 * - Digital Input 1: Pin D2
 * - Digital Input 2: Pin D3
 * - Digital Output 1: Pin D6 (Mirrors Node 1's Input 1)
 * - Digital Output 2: Pin D7 (Mirrors Node 1's Input 2)
 * - Analog Input 1: Pin A0
 * - Analog Output 1 (PWM): Pin D10 (Mirrors Node 1's Analog Input 1)
 */

#include <BeeBusGlobal.h>

// =============================================================================
// CONFIGURATION - CHANGE THESE FOR EACH NODE
// =============================================================================
#define NODE_ID 2
#define REMOTE_NODE_ID 1

// =============================================================================
// LINE, PINS AND REGISTER MAP - ADJUST THE PINS FOR YOUR BOARD
// =============================================================================
#define SERIAL_BAUD 115200
#define BEEBUS_BAUD 115200

// --- Pin Definitions ---
// Inputs
const int DIGITAL_INPUT_1_PIN = 2;
const int DIGITAL_INPUT_2_PIN = 3;
const int ANALOG_INPUT_1_PIN = A0;
// Outputs
const int DIGITAL_OUTPUT_1_PIN = 6;
const int DIGITAL_OUTPUT_2_PIN = 7;
const int ANALOG_OUTPUT_1_PIN = 10; // Must be a PWM pin

// --- Modbus Register Addresses ---
// These addresses are for THIS node's outputs. The remote node will write to them.
const uint16_t MY_DIGITAL_OUTPUTS_ADDR = 0; // Coils
const uint16_t MY_ANALOG_OUTPUTS_ADDR = 0;  // Holding Registers

// These addresses are for the REMOTE node's outputs. This node will write to them.
const uint16_t REMOTE_DIGITAL_OUTPUTS_ADDR = 0; // Coils
const uint16_t REMOTE_ANALOG_OUTPUTS_ADDR = 0;  // Holding Registers

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
BeeBusAPI beebus;

// --- I/O State Variables ---
// My Inputs: We read these from our physical pins.
bool my_digital_input_1 = false;
bool my_digital_input_2 = false;
int16_t my_analog_input_1 = 0;

// My Outputs: The remote node writes to these variables via Modbus.
bool my_digital_output_1 = false;
bool my_digital_output_2 = false;
int16_t my_analog_output_1 = 0;

// Timing for the main loop
unsigned long last_update_time = 0;
const unsigned long UPDATE_INTERVAL_MS = 100;

void printStatus();

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial);
    Serial.println("### BeeBus Simple I/O Example - Node 2 ###");

    // --- Configure Hardware Pins ---
    pinMode(DIGITAL_INPUT_1_PIN, INPUT_PULLUP);
    pinMode(DIGITAL_INPUT_2_PIN, INPUT_PULLUP);
    pinMode(ANALOG_INPUT_1_PIN, INPUT);
    pinMode(DIGITAL_OUTPUT_1_PIN, OUTPUT);
    pinMode(DIGITAL_OUTPUT_2_PIN, OUTPUT);
    pinMode(ANALOG_OUTPUT_1_PIN, OUTPUT);

    // --- Initialise BeeBus ---
    // The baud rate is passed too: it is the one property of the line a node
    // cannot measure for itself.
    Serial1.begin(BEEBUS_BAUD);
    beebus.begin(&Serial1, NODE_ID, BeeBusConfig(), BEEBUS_BAUD);

    // --- Bind MY OUTPUT variables to the Modbus Data Map ---
    // This allows the remote node to write to our output variables.
    beebus.addCoil(MY_DIGITAL_OUTPUTS_ADDR + 0, &my_digital_output_1);
    beebus.addCoil(MY_DIGITAL_OUTPUTS_ADDR + 1, &my_digital_output_2);
    beebus.addHreg(MY_ANALOG_OUTPUTS_ADDR + 0, &my_analog_output_1);

    // begin() only prepares the stack. connect() starts the join sequence.
    beebus.connect();

    Serial.println("Node 2 initialized. Waiting for remote node...");
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
    // The BeeBus state machine must be called every loop to process messages.
    beebus.loop();

    // Only run the logic at a fixed interval.
    if (millis() - last_update_time < UPDATE_INTERVAL_MS) {
        return;
    }
    last_update_time = millis();

    // --- Step 1: Read my local physical inputs ---
    my_digital_input_1 = !digitalRead(DIGITAL_INPUT_1_PIN); // Invert for PULLUP
    my_digital_input_2 = !digitalRead(DIGITAL_INPUT_2_PIN); // Invert for PULLUP
    my_analog_input_1 = analogRead(ANALOG_INPUT_1_PIN);

    // --- Step 2: Write my inputs to the remote node's outputs ---
    // Check if the other node is on the network before trying to write.
    if (beebus.isMember(REMOTE_NODE_ID)) {
        // Create temporary arrays to hold the values we want to write.
        bool digital_values_to_write[2] = {my_digital_input_1, my_digital_input_2};
        int16_t analog_values_to_write[1] = {my_analog_input_1};

        // Write the values. These return a BeeBusResult; check it with
        // bb_accepted() if you want to react to a rejected operation.
        beebus.writeCoil(REMOTE_NODE_ID, REMOTE_DIGITAL_OUTPUTS_ADDR, digital_values_to_write);
        beebus.writeHreg(REMOTE_NODE_ID, REMOTE_ANALOG_OUTPUTS_ADDR, analog_values_to_write);
    }

    // --- Step 3: Update my local physical outputs ---
    // The `my_digital_output_...` variables are updated in the background by
    // beebus.loop() when the remote node writes to us. We just need to
    // apply those values to our physical output pins.
    digitalWrite(DIGITAL_OUTPUT_1_PIN, my_digital_output_1);
    digitalWrite(DIGITAL_OUTPUT_2_PIN, my_digital_output_2);
    // Scales a 10-bit reading to 8-bit PWM; adjust for your board's ADC and
    // PWM resolution.
    analogWrite(ANALOG_OUTPUT_1_PIN, my_analog_output_1 / 4);

    // --- Step 4: Print Status ---
    printStatus();
}

// =============================================================================
// UTILITY FUNCTION
// =============================================================================
void printStatus() {
    Serial.println("--- Node 2 Status ---");
    Serial.print("My Inputs:  DI_1=");
    Serial.print(my_digital_input_1);
    Serial.print(", DI_2=");
    Serial.print(my_digital_input_2);
    Serial.print(" | AI_1=");
    Serial.println(my_analog_input_1);

    Serial.print("My Outputs: DO_1=");
    Serial.print(my_digital_output_1);
    Serial.print(", DO_2=");
    Serial.print(my_digital_output_2);
    Serial.print(" | AO_1=");
    Serial.println(my_analog_output_1);
    Serial.println("---------------------\n");
}
