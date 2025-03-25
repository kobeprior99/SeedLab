#include <Wire.h>

#define I2C_ADDR 0x08
float receivedData[6];  // Store received float values

void receiveData(int byteCount) {
    if (byteCount < 24) return;  // Ensure we receive exactly 6 floats (6 * 4 bytes)

    byte floatBytes[4];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 4; j++) {
            if (Wire.available()) {
                floatBytes[j] = Wire.read();  // Read 4 bytes per float
            }
        }
        memcpy(&receivedData[i], floatBytes, sizeof(float));  // Convert bytes to float
    }

    // Debug: Print received data
    Serial.print("Received: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(receivedData[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void setup() {
    Wire.begin(I2C_ADDR);
    Wire.onReceive(receiveData);
    Serial.begin(115200);
}

void loop() {
    // Waiting for I2C data
}
