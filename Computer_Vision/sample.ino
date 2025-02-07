#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;
volatile uint8_t coordinates[2] = {}; 

void setup() {
  // put your setup code here, to run once:
  //address
  Wire.begin(MY_ADDR);
  //when receiving call reeceive function
  Wire.onReceive(receive);
 
}

//in the loop just read from coordinate array
//left wheel coordinates[0], right wheel coordinates[1]

void receive(){
  offset = Wire.read();//even though we don't use the offset you have to read this data
//   fill the coordinate array when data available on wire
  for(int i = 0; i< 2 && Wire.available(); i++){
    coordinates[i] = Wire.read();
  }
  }
