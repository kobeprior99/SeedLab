//SEED Assignment 2 - Part 2b
//Localization and Controls
//Cooper Hammond

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

//only pins 2 and 3 have ISR functions, so update code for ISRs to make motor 1 and motor 2 use the pins and work together

// Timing variables
unsigned long desired_Ts_ms = 10;  // Desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

//Variables for calculating position
long pos1_counts;
long pos2_counts;

const float circumference = 2 * PI * (5.93/2); //distance: Measured 5.93in diameter -> c=2(pi)r
const float b = 12; //wheel base 12inches

void setup() {
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

  // Initialize Serial communication
  Serial.begin(115200); // Set baud rate to 115200 for fast data output

  // Initialize timing
  last_time_ms = millis();  // Record the start time for the first sample
  start_time_ms = last_time_ms;

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(encPin1A), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPin2A), encoder2_ISR, CHANGE);
}

// Interrupt service routines (ISR) for counting encoder pulses
void encoder1_ISR() {
  if (digitalRead(encPin1A) == digitalRead(encPin1B)) {
    pos1_counts-=2; // Clockwise rotation
  } else {
    pos1_counts+=2; // Counter-clockwise rotation
  }
}

void encoder2_ISR() {
  if (digitalRead(encPin2A) == digitalRead(encPin2B)) {
    pos2_counts+=2; // Counter-clockwise rotation
  } else {
    pos2_counts-=2; // Clockwise rotation
  }
}

//last position initialized
float lastXPos = 0.0, lastYPos = 0.0, lastAngle = 0.0;

// Timing variable for data reading incrementation
unsigned long last_read_time = 0;
const unsigned long read_interval = 100; 

void loop() {
  // Check if time interval has passed
  if (millis() - last_read_time >= read_interval) {
    last_read_time = millis(); // Update last read time

    // Calculate how far each wheel has traveled linearly in inches
    float wheel1Dist = ((float)pos1_counts / 3200) * circumference;
    float wheel2Dist = ((float)pos2_counts / 3200) * circumference;

    // Check if movement occurred
    bool isMoving = (abs(wheel1Dist) > 0.001 || abs(wheel2Dist) > 0.001);

    // Variables to store the updated position and angle
    float currentXPos = lastXPos;
    float currentYPos = lastYPos;
    float currentAngle = lastAngle;

    // If movement detected, update position and angle
    if (isMoving) {
      currentAngle = lastAngle + ((wheel1Dist - wheel2Dist) / b);
      currentXPos = lastXPos + cos(lastAngle) * ( (wheel1Dist + wheel2Dist) / 2 );
      currentYPos = lastYPos + sin(lastAngle) * ( (wheel1Dist + wheel2Dist) / 2 );

      // Update last known values
      lastXPos = currentXPos;
      lastYPos = currentYPos;
      lastAngle = currentAngle;

      // Reset encoder counts to avoid accumulation errors
      pos1_counts = 0;
      pos2_counts = 0;
    }

    // Convert angle to degrees for display
    float currentAngleDeg = currentAngle * (180 / PI);

    // Calculate elapsed time in seconds
    current_time = (float)(millis() - start_time_ms) / 1000.0;

    //Serial print values with labels
    Serial.print("Time: ");
    Serial.print(current_time);
    Serial.print("\t xPos: ");
    Serial.print(currentXPos);
    Serial.print("\t yPos: ");
    Serial.print(currentYPos);
    Serial.print("\t phi: ");
    Serial.print(currentAngleDeg);
    Serial.print("\n");
    
/*
    //Serial print for matlab data
    Serial.print(current_time);
    Serial.print(" ");
    Serial.print(currentXPos);
    Serial.print(" ");
    Serial.print(currentYPos);
    Serial.print(" ");
    Serial.print(currentAngleDeg);
    Serial.print(";\n");
*/
  }
}