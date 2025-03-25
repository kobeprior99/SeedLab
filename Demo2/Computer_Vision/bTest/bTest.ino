#include <Wire.h>
#define MY_ADDR 8

const int BUFFER_SIZE = 24; 
volatile float instruction_array[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void setup() {
    Serial.begin(115200);
    Wire.begin(MY_ADDR);
    Wire.onReceive(receive);
}

void receive(int numBytes) {
    if (numBytes == BUFFER_SIZE + 1) {
        Wire.read();  
        byte buffer[BUFFER_SIZE];
        if (Wire.available() >= BUFFER_SIZE) {
            Wire.readBytes(buffer, BUFFER_SIZE);
            memcpy((void*)instruction_array, buffer, BUFFER_SIZE);
        }
    }
    
    Serial.println("Received instructions:");
    Serial.print("Angle: "); Serial.println(instruction_array[0] ? instruction_array[1] : "N/A");
    Serial.print("Distance: "); Serial.println(instruction_array[2] ? instruction_array[3] : "N/A");
    Serial.print("Arrow: "); Serial.println(instruction_array[4] ? instruction_array[5] : "N/A");
}

void loop() {
    delay(500);
}
