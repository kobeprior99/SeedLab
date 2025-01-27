#include <Wire.h>
#define MY_ADDR 8

volatile int receivedInt = 0;
volatile int modifiedInt = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);
  Wire.onRequest(request);
}

void loop() {}

void receive() {
  if (Wire.available()) {
    receivedInt = Wire.read(); // Read 1 byte
    modifiedInt = receivedInt + 100; // Add 100 to the received byte

    // Debugging: Print received and modified integer to Serial Monitor
    Serial.print("Received: ");
    Serial.println(receivedInt);
    Serial.print("Modified: ");
    Serial.println(modifiedInt);
  }
}

void request() {
  // Send the modified integer back (1 byte)
  Wire.write(modifiedInt); // Send modified value
}
