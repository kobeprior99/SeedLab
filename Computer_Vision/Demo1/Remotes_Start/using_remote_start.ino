/*
 * File: using_remote_start.ino
 * Author: Kobe Prior
 * Date: 02/22/2025
 * Description: This file contains the code to start the main loop of the program 
 *              upon receiving instructions via I2C communication. The device 
 *              listens on address MY_ADDR and processes incoming data to trigger 
 *              the main loop.
 */
#include <Wire.h>
#define MY_ADDR 8

// set up flag so that the main loop only starts when instructions are received
boolean start = false;
volatile uint8_t offset = 0;
// instruction[0] = distance, instruction[1] = angle
volatile uint8_t instructions[2] = {}; 

void receive(){
    offset = Wire.read();//even though we don't use the offset you have to read this data
    //fill the coordinate array when data available on wire
    for(int i = 0; i < 2 && Wire.available(); i++){
      instructions[i] = Wire.read();
    }
    // start the main loop
    start = true;
    }

void setup() {
  // put your setup code here, to run once:
  //address
  Wire.begin(MY_ADDR);
  //when receiving call reeceive function
  Wire.onReceive(receive);
}



// start main loop
void loop() {
    while (!start){
        // wait for instruction
        //this loop will not be reentered until start is set to false (happnens at the beginning of the program
    }

    // put your main code here, to run repeatedly:
}


