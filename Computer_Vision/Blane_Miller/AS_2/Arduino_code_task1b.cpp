#include <Wire.h>
#define MY_ADDR 8

volatile int receivedInt = 0;  // Variable to store the received integer
volatile int modifiedInt = 0;  // Variable to store the modified integer

void setup() {
  Serial.begin(115200);        // Initialize Serial communication
  Wire.begin(MY_ADDR);         // Join I2C bus as a slave with address MY_ADDR
  Wire.onReceive(receive);     // Register function to handle data reception
  Wire.onRequest(request);     // Register function to handle data request
}

void loop() {
  // Nothing to do here since I2C interrupts handle communication
}

void receive() {
  if (Wire.available() >= 1) {
    // Read the integer sent by the Raspberry Pi
    receivedInt = Wire.read();

    // Add 100 to the received integer
    modifiedInt = receivedInt + 100;

    // For debugging: Print values to Serial Monitor
    Serial.print("Received Integer: ");
    Serial.println(receivedInt);
    Serial.print("Modified Integer: ");
    Serial.println(modifiedInt);
  }
}

void request() {
  // Send the modified integer back to the Raspberry Pi
  Wire.write(modifiedInt);
}
