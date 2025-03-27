
/**
 * @file Test_Send.ino
 * @author Kobe Prior
 * @date March 24, 2025
 * @brief Demonstrates functionality of I2C communication.
 *
 * This program sets up an Arduino device to act as an I2C slave with a specific address.
 * It listens for data sent from an I2C master device and processes the received data.
 * The received data includes a flag for valid angle and distance, an arrow indicator, 
 * and two floating-point values representing angle and distance. The processed data 
 * is then printed to the Serial Monitor for debugging or further use.
 *
 * Features:
 * - Configures the device as an I2C slave with a clock speed of 400 kHz.
 * - Receives and processes data sent by an I2C master.
 * - Stores byte data into 5 variables.
 * - Outputs the processed data to the Serial Monitor.
 */
#include <Wire.h>
#define MY_ADDR 8

// set up flag so that the main loop only starts when instructions are received
volatile uint8_t offset = 0;
const int BUFFER_SIZE = 4;  //four bytes for float
byte buffer[BUFFER_SIZE]; // a buffer used to store bytes to recombine iinto float

//variables that will be written to on receive
volatile float angle = 0.0;  //array to store the instructions
volatile float distance = 0.0;
volatile int good_angle = 0;
volatile int good_distance = 0;
volatile int arrow = 2;

volatile bool newData = false; //used for controlling print 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //address
  Wire.setClock(400000);  // 400 kHz clock speed
  Wire.begin(MY_ADDR);
  //when receiving call reeceive function
  Wire.onReceive(receive);
}

void receive() {
  while (Wire.available()) {
    Wire.read();  //discard first byte (offset)
    good_angle = Wire.read(); // 1 byte for good angle: 0 valid 1 invalid
    good_distance = Wire.read(); // 1 byte for good distance: 0 valid 1 invalid
    arrow = Wire.read(); // 1 byte for arrow 0 left, 1 right, 2 no arrow
    //need to read four bytes and convert into float
    for (int i = 0; i < BUFFER_SIZE; i++) {
      buffer[i] = Wire.read();
    }
    memcpy(&angle, buffer, sizeof(angle));//create float from 4 bytes and copy it into angle var

    for (int i = 0; i < BUFFER_SIZE; i++) {
      buffer[i] = Wire.read();
    }
    memcpy(&distance, buffer, sizeof(distance)); // create float from 4 bytes and copy it into angle var
    newData = true;//to control print
  }
}

void loop() {
  //debug to see data come through, can lower delay to 10 to reflect the refresh rate of the control system and it continues to work.
  if (newData) {
    newData = false;
    // example 0, 0, 2, 4, 12.5 means angle and distance are invalid and there is no arrow
    // example 1, 1, 0, -0.67, 12 means angle is valid and -.67 deg, distance is valid and 12 inches, and there is a left arrow detected
    Serial.print(good_angle);
    Serial.print(", ");
    Serial.print(good_distance);
    Serial.print(", ");
    Serial.print(arrow);
    Serial.print(", ");
    Serial.print(angle);
    Serial.print(", ");
    Serial.println(distance);
    delay(1000);//small delay so we don't print too much and needlessly crowd serial monitor
  }
}