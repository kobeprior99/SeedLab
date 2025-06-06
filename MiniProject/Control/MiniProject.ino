/*******************************************************************
* File Name         : MiniProject.ino
* Description       : Move left and right wheel 180 degrees depending on quadrant of aruco marker
*                    
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 02/05/2025	Cooper Hammond and Ron Gaines	Created File
*
*Supplementary files: Control/Cooper_Hammond/miniProject_CommMove.txt
Control/Ron_Gaines/Control Values
Control/MiniProject/Simulink folder contains all values for simulink along with instructions
******************************************************************
Hardware Setup:  Place LCD header on Pi, connect following headers to Arduino:
-connect gnd to gnd, sda to sda, and scl to scl from Pi to Arduino.
-connect pin 3 on Pi to pin A4 on Arduino (SDA)
-connect pin 5 on Pi to pin A5 on Arduino (SCL)
-connect pin 6 on Pi to GND on Arduino
Connect camera to the Pi
Example Execution: upload this ino file to the arduino and push it to the board. 
On the Raspberry Pi follow the instructions on the MiniProject.py file to run it
Prop the robot on its center of mass so the wheels can spin freely
******************************************************************/


// Quadrant

// 0,0 | 1,0
// ----|----
// 0,1 | 1,1

//PI communication Initialization
#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;

// Pin definitions
const int enablePin = 4;    // Pin 4 to enable the motor driver
const int pwmPin[2] = {9, 10};      // Pin 9 for PWM control (Motor 1), Pin 10 for PWM control (Motor 2)
const int dirPin[2] = {7, 8};      // Pin 7 for direction control (Motor 1), Pin 8 for direction control (Motor 2)

// Encoder pins (one per motor)
const int encPin1A = 2; // Encoder pin A for motor 1
const int encPin2A = 3; // Encoder pin A for motor 2
const int encPin1B = 5; // Encoder pin B for motor 1
const int encPin2B = 6; // Encoder pin B for motor 2

// Timing variables
unsigned long desired_Ts_ms = 10;  // Desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

//Array variables [motor1, motor2]
  //volatile
volatile int piCommand[2] = {0, 0};
volatile int prevPiCommand[2] = {0, 0};
volatile int pos_counts[2] = {0, 0};
volatile int prev_counts[2] = {0, 0};
  //set in error correction
float pos_error[2] = {0, 0};
float integral_error[2] = {0, 0};
float desired_speed[2] = {0, 0};
float error[2] = {0, 0};
float Voltage[2] = {0, 0};
  //calculations
float desired_pos[2] = {0, 0}; //need to write code for
float actual_pos[2] = {0, 0}; //is pos_counts in radians
float prev_actual_pos[2] = {0, 0}; //is prev_counts in radians
float delta_pos[2] = {0, 0};
float actual_speed[2] = {0, 0};
float PWM[2] = {0, 0};

//Array Constants [motor1, motor2]
const float K[2] = {1.5, 1.75};
const float sigma[2] = {14, 14};
const float Kp[2] = {1.75, 2};
<<<<<<< HEAD:Control/Cooper_Hammond/miniProject.txt
const float P[2] = {4.2, 4};
const float I[2] = {0.1, .1};
=======
//const float P[2] = {23.3173903205445, 26.9494689554926};
//const float I[2] = {81.8636132310649, 117.453423846055};
const float P[2] = {4.2, 4};
const float I[2] = {0.1, 0.1};
>>>>>>> 197556670d4cbc076bf1845feb3c388e70e91bbb:MiniProject/Control/MiniProject.ino

//Constants
const float Battery_Voltage = 7.8;

//Boolean Values
boolean go = false;

void setup() {
   // Initialize Serial communication
  Serial.begin(115200); // Set baud rate to 115200 for fast data output
  
  //PI Communication Setup
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);
  Serial.println("Arduino ready to receive data:");
  
  // Set motor control pins as outputs
  pinMode(enablePin, OUTPUT);
  pinMode(pwmPin[0], OUTPUT);
  pinMode(pwmPin[1], OUTPUT);
  pinMode(dirPin[0], OUTPUT);
  pinMode(dirPin[1], OUTPUT);

  // Set encoder pins as inputs
  pinMode(encPin1A, INPUT);
  pinMode(encPin2A, INPUT);
  
  // Enable the motor driver by setting the enable pin HIGH
  digitalWrite(enablePin, HIGH);

  // Initialize timing
  last_time_ms = millis();  // Record the start time for the first sample
  start_time_ms = last_time_ms;

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(encPin1A), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPin2A), encoder2_ISR, CHANGE);
}

//Funtions

//PI Communication Function
void receive() {
  offset = Wire.read();
  for (int i = 0; i<2 && Wire.available(); i++){
      piCommand[i] = Wire.read();
    }  
}

// Interrupt service routines (ISR) for counting encoder pulses
void encoder1_ISR() {
  if (digitalRead(encPin1A) == digitalRead(encPin1B)) {
    pos_counts[0] -= 2; // Clockwise rotation
  } else {
    pos_counts[0] += 2; // Counter-clockwise rotation
  }
}

void encoder2_ISR() {
  if (digitalRead(encPin2A) == digitalRead(encPin2B)) {
    pos_counts[1] += 2; // Counter-clockwise rotation
  } else {
    pos_counts[1] -= 2; // Clockwise rotation
  }
}


void loop() {
  
  receive(); // Get updated commands from Raspberry Pi

  // Print only when piCommand changes
  if ( (piCommand[0] != prevPiCommand[0]) || (piCommand[1] != prevPiCommand[1]) ) {
    Serial.print("Sector: (");
    Serial.print(piCommand[0]);
    Serial.print(", ");
    Serial.print(piCommand[1]);
    Serial.println(") ");

    Serial.print("Positions: ");
    Serial.print(pos_counts[0]);
    Serial.print(" ");
    Serial.println(pos_counts[1]);
    
    go = true;
  }

  
  // Motor Control Logic
    //Motor 1
  if ( (pos_counts[0] < (prev_counts[0] + 1600)) && (pos_counts[0] > (prev_counts[0] - 1600)) && go == true) {
    if(piCommand[0] != prevPiCommand[0]){
      desired_pos[0] = (2 * PI * ((piCommand[0] * 1600)+1) / 3200);
      
    }
  } else {
    go = false;
  }
    //Motor 2
  if ( (pos_counts[1] < (prev_counts[1] + 1600)) && (pos_counts[1] > (prev_counts[1] - 1600)) && go == true) {
    if(piCommand[1] != prevPiCommand[1]){
      desired_pos[1] = (2 * PI * ((piCommand[1] * 1600)+1) / 3200);
      
    }
  } else {
    go = false;
  }

  
   for(int i=0;i<2;i++) {
 
    actual_pos[i] = 2 * PI * (float)pos_counts[i] / 3200;
    prev_actual_pos[i] = 2 * PI * (float)prev_counts[i] / 3200;
    delta_pos[i] = actual_pos[i] - prev_actual_pos[i];
    actual_speed[i] = delta_pos[i] / ((float)desired_Ts_ms / 1000);
    //desired_pos[i] = (2 * PI * (pi Command[i] * 1600) / 3200);

    //given error code
    pos_error[i] = desired_pos[i] - actual_pos[i];
    integral_error[i] = integral_error[i] + pos_error[i]*((float)desired_Ts_ms /1000);
    desired_speed[i] = (P[i] * pos_error[i]) + (I[i] * integral_error[i]);
    error[i] = desired_speed[i] - actual_speed[i];
    Voltage[i] = Kp[i] * error[i];

    PWM[i] = 255 * abs(Voltage[i]) / Battery_Voltage;
     
    if(pos_error[i] > 0){
      digitalWrite( dirPin[i], HIGH );
    }
    else{
      digitalWrite( dirPin[i], LOW );
    }
    
    analogWrite( pwmPin[i], min(PWM[i], 255));
    
  }

  //Reset piCommand
  prevPiCommand[0] = piCommand[0];
  prevPiCommand[1] = piCommand[1];

  prev_counts[0] = pos_counts[0];
  prev_counts[1] = pos_counts[1];
  
  // Wait until the next sample period
  while (millis() < last_time_ms + desired_Ts_ms) {
    // Idle loop to maintain timing
  }

  last_time_ms = millis(); // Update last timestamp
 
//digitalWrite( dirPin[0], HIGH );
//digitalWrite( dirPin[1], HIGH );
//
//analogWrite( pwmPin[0], 50);  
//analogWrite( pwmPin[1], 50); 
} //end void loop
