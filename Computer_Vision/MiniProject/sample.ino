/**
 * @file sample.ino
 * @brief Sample code for the control team to read quadrant coordinates.
 * 
 * This file contains sample code to demonstrate how to read quadrant coordinates
 * using the Wire library for I2C communication. The code sets up an I2C slave device
 * with a specific address and reads coordinate data from the I2C bus.
 * 
 * The coordinates are stored in a volatile array and can be accessed in the main loop.
 * The left wheel coordinate is stored in coordinates[0], and the right wheel coordinate
 * is stored in coordinates[1].
 * 
 * @author 'Kobe Prior'
 * @date Feb 7, 2025
 */

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
  //fill the coordinate array when data available on wire
  for(int i = 0; i < 2 && Wire.available(); i++){
    coordinates[i] = Wire.read();
  }
  }
