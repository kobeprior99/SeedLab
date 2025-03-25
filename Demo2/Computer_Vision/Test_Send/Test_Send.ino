#include <Wire.h>
#define MY_ADDR 8

// set up flag so that the main loop only starts when instructions are received
volatile uint8_t offset = 0;
// instruction[0] = distance, instruction[1] = angle
const int BUFFER_SIZE = 4; //four bytes for float
byte buffer[BUFFER_SIZE];
volatile float angle = 0.0; //array to store the instructions
volatile float distance = 0.0;
volatile int good_angle = 0;
volatile int good_distance = 0;
volatile int arrow = 2;

volatile bool newData = false;
//declared gloabaly to avoid reallocating memory each time receive is called
void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  //address
  Wire.setClock(400000); 
  Wire.begin(MY_ADDR);
  //when receiving call reeceive function
  Wire.onReceive(receive);

}
void receive(int numBytes){
    if (Wire.available() ){
      Wire.read(); //discard first byte (offset)
      good_angle = Wire.read();
      good_distance = Wire.read();
      arrow = Wire.read();
      //need to read four bytes and convert into float
      for (int i = 0; i < BUFFER_SIZE; i++){
        buffer[i] = Wire.read();
      }
      memcpy(&angle, buffer, sizeof(angle));
      
      for (int i = 0; i < BUFFER_SIZE; i++){
        buffer[i] = Wire.read();
      }
      memcpy(&distance, buffer, sizeof(distance));

      //memcpy(instruction_array, buffer, BUFFER_SIZE);
      newData = true;
    }
}

void loop() {
  if (newData){
    newData = false;
    // put your main code here, to run repeatedly:
    // float good_angle = instruction_array[0];
    // float angle = instruction_array[1];
    // float good_distance = instruction_array[2];
    // float distance = instruction_array[3];
    // float good_arrow = instruction_array[4];
    // float arrow = instruction_array[5];

    // Serial.println("Received instructions:");
    // Serial.print("Angle: ");
    // if(good_angle == 1.0) Serial.println(angle);
    // else Serial.println("N/A");
    // Serial.print("Distance: ");
    // if(good_distance == 1.0) Serial.println(distance);
    // else Serial.println("N/A");
    // Serial.print("Arrow: ");
    // if(good_arrow == 1.0) Serial.println(arrow);
    // else Serial.println("N/A");
    // delay(10);
  }
  
}
