#include <Wire.h>
#define MY_ADDR 8

// set up flag so that the main loop only starts when instructions are received
volatile uint8_t offset = 0;
// instruction[0] = distance, instruction[1] = angle
const int BUFFER_SIZE = 25; //6 floats*4 bytes each 24 bytes + 1 byte offset
volatile float instruction_array[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //array to store the instructions
volatile bool newData = false;
//declared gloabaly to avoid reallocating memory each time receive is called
volatile byte buffer[BUFFER_SIZE];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //address
  Wire.begin(MY_ADDR);
  //when receiving call reeceive function
  Wire.onReceive(receive);
}
void receive(int numBytes){
    if (numBytes == BUFFER_SIZE){
      Wire.read(); //discard first byte (offset)
      Wire.readBytes(buffer, BUFFER_SIZE);
      memcpy(instruction_array, buffer, BUFFER_SIZE);
      newData = true;
    }
    }

void loop() {
  if (newData){
    newData = false;
    // put your main code here, to run repeatedly:
    float good_angle = instruction_array[0];
    float angle = instruction_array[1];
    float good_distance = instruction_array[2];
    float distance = instruction_array[3];
    float good_arrow = instruction_array[4];
    float arrow = instruction_array[5];

    Serial.println("Received instructions:");
    Serial.print("Angle: ");
    if(good_angle == 1.0) Serial.println(angle);
    else Serial.println("N/A");
    Serial.print("Distance: ");
    if(good_distance == 1.0) Serial.println(distance);
    else Serial.println("N/A");
    Serial.print("Arrow: ");
    if(good_arrow == 1.0) Serial.println(arrow);
    else Serial.println("N/A");
    delay(10);
  }
  
}
