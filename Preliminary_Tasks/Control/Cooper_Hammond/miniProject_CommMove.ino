//SEED MINI PROJECT - PI Arduinio Communication Translated to Movement
//Localization and Controls
//Cooper Hammond

// 0,0 | 1,0
// ----|----
// 0,1 | 1,1

//PI communication Initialization
#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;

// Pin definitions
const int enablePin = 4;    // Pin 4 to enable the motor driver
const int pwmPin1 = 9;      // Pin 9 for PWM control (Motor 1)
const int pwmPin2 = 10;     // Pin 10 for PWM control (Motor 2)
const int dirPin1 = 7;      // Pin 7 for direction control (Motor 1)
const int dirPin2 = 8;      // Pin 8 for direction control (Motor 2)

// Encoder pins (one per motor)
const int encPin1A = 2; // Encoder pin A for motor 1
const int encPin2A = 3; // Encoder pin A for motor 2
const int encPin1B = 5; // Encoder pin B for motor 1
const int encPin2B = 6; // Encoder pin B for motor 2

//Array variables [motor1, motor2]
volatile int piCommand[2] = {0,0};
volatile int prevPiCommand[2] = {0,0};

// Timing variables
unsigned long desired_Ts_ms = 100;  // Desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

//Variables for calculating position
volatile int pos_counts[2] = {0, 0}; 
volatile int prev_counts[2] = {0, 0};

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
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);

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

//PI Communication Funtions
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

    prev_counts[0] = pos_counts[0];
    prev_counts[1] = pos_counts[1];
    
    go = true;
  }

  // Motor Control Logic
  if ( (pos_counts[0] < (prev_counts[0] + 1600)) && (pos_counts[0] > (prev_counts[0] - 1600)) && go == true) {
    if(piCommand[0] != prevPiCommand[0]){
      digitalWrite( dirPin1, HIGH );
      analogWrite( pwmPin1, 50);
    }
  } else {
    analogWrite(pwmPin1, 0);
    go = false;
  }

  if ( (pos_counts[1] < (prev_counts[1] + 1600)) && (pos_counts[1] > (prev_counts[1] - 1600)) && go == true) {
    if(piCommand[1] != prevPiCommand[1]){
      digitalWrite( dirPin2, HIGH );
      analogWrite( pwmPin2, 50);
    }
  } else {
    analogWrite(pwmPin2, 0);
    go = false;
  }

  prevPiCommand[0] = piCommand[0];
  prevPiCommand[1] = piCommand[1];
  
  // Wait until the next sample period
  while (millis() < last_time_ms + desired_Ts_ms) {
    // Idle loop to maintain timing
  }

  last_time_ms = millis(); // Update last timestamp
  
}