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
  int i=0; //reset index
  //   clear index coord
  coordinates = {}
  offset = Wire.read();//even though we don't use the offset you have to read this data
  while(Wire.available()){
    //populate coordinate (characters)
    coordinates[i] = Wire.read();
    i++;
    }
  }
