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
const int BUFFER_SIZE = 24; //6 floats*4 bytes each
volatile float instruction_array[6]; //array to store the instructions



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
      byte buffer[BUFFER_SIZE];
      Wire.readBytes(buffer, BUFFER_SIZE);
      memcpy((void*)instruction_array, buffer, BUFFER_SIZE);

      //debug:
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

      //start the rest of the code
      start = true;
    }

    }


// start main loop
void loop() {
    while (!start){
        // wait for instruction
        //this loop will not be reentered until start is set to false (happnens at the beginning of the program
    }
    // put your main code here, to run repeatedly:
    Serial.println("hello every 2 seconds");
    delay(2000);
}
