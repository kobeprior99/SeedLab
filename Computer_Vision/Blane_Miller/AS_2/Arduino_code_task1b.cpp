#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;
volatile int receivedInt = 0;  // Variable to store received integer
volatile int modifiedInt = 0;  // Variable to store the modified integer
volatile uint8_t reply = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);
  Wire.onRequest(request);
}

void loop() {
  // Check if there is a received message
  if(receivedInt != 0){
    // Add 100 to the received integer
    modifiedInt = receivedInt + 100;
    
    // Optionally: Turn the LED on if modified integer is above a threshold
    if(modifiedInt > 100){
      digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
    } else {
      digitalWrite(LED_BUILTIN, LOW); // Turn off LED
    }

    // Print the modified integer to the serial monitor (for debugging)
    Serial.print("Received Integer: ");
    Serial.println(receivedInt);
    Serial.print("Modified Integer: ");
    Serial.println(modifiedInt);
    
    // Reset receivedInt to indicate the message was processed
    receivedInt = 0;
  }
}

void receive(){
  // Read the received integer (assuming it's sent as 1 byte)
  offset = Wire.read();
  if(Wire.available()){
    receivedInt = Wire.read();  // Read the integer
  }
}

void request(){
  // Send back the modified integer to the Raspberry Pi
  Wire.write(modifiedInt);  // Send modified integer
  reply = 0;  // Reset reply variable
}
