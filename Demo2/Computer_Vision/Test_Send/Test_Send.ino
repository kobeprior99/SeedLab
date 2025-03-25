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
  Serial.begin(9600);
  //address
  Wire.setClock(400000); // 400 kHz clock speed
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
  Serial.print(good_angle); Serial.print(", ");
  Serial.print(good_distance); Serial.print(", ");
  Serial.print(arrow); Serial.print(", ");
  Serial.print(angle); Serial.print(", ");
  Serial.println(distance);;

  delay(1000);
}
}