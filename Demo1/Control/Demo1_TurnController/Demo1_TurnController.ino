// SEED DEMO #1  *no PI communication
// THIS CODE RUNS ON PWMS AND SUCKS
// Localization and Controls
// Cooper Hammond and Ron Gaines

//work on intergrating static and turn contols systems: look at desired_pos[]

// Demo 1 Instructions
const float target_angle = 360; //angle to turn in degrees

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
unsigned long desired_Ts_ms = 50;  // Desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

// Array variables [motor1, motor2]
  //volatile
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

// Array Constants [motor1, motor2]
const float K[2] = {1.5, 1.75};
const float sigma[2] = {14, 14};
const float Kp[2] = {1.75, 2};
const float P[2] = {4.2, 4};
const float I[2] = {0.2, 0.2};

// Constants
const float Battery_Voltage = 7.8;
const float circumference = PI * 5.93; //distance: Measured 5.93in diameter -> c=2(pi)r -> 18.63
const float b = 12; //wheel base 12inches

// Position initialized
float lastXPos = 0.0, lastYPos = 0.0, lastAngle = 0.0;
float currentPos = 0.0;
float currentAngleDeg = 0.0;

// Timing variable for data reading incrementation
unsigned long last_read_time = 0;
const unsigned long read_interval = 100; 

// Overall Position
float robot_pos = 0;
volatile bool moveCommand = true; //stopped == false, driving == true

void setup() {
  // Initialize Serial communication
  Serial.begin(115200); // Set baud rate to 115200 for fast data output
  Serial.println("Ready to fuck some shit up!!!");

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

// Functions

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

  // Check if time interval has passed
  if (millis() - last_read_time >= read_interval) {
    last_read_time = millis(); // Update last read time

    // Calculate elapsed time in seconds
    current_time = (float)(millis() - start_time_ms) / 1000.0;
    
    // Calculate how far each wheel has traveled linearly in inches
    float wheel1Dist = ((float)(pos_counts[0]-prev_counts[0]) / 3200) * circumference;
    float wheel2Dist = ((float)(pos_counts[1]-prev_counts[1]) / 3200) * circumference;

    // Check if movement occurred
    bool isMoving = (abs(wheel1Dist) > 0.001 || abs(wheel2Dist) > 0.001);
  
    // Variables to store the updated position and angle
    float currentXPos = lastXPos;
    float currentYPos = lastYPos;
    float currentAngle = lastAngle;
  
    // If movement detected, update position and angle
    if (isMoving) {
      currentAngle = lastAngle + ((wheel2Dist - wheel1Dist) / b);
      currentXPos = lastXPos + cos(lastAngle) * ( (wheel1Dist + wheel2Dist) / 2 );
      currentYPos = lastYPos + sin(lastAngle) * ( (wheel1Dist + wheel2Dist) / 2 );
  
      // Update last known values
      lastXPos = currentXPos;
      lastYPos = currentYPos;
      lastAngle = currentAngle;
    }
      
      // Convert angle to degrees for display
      currentAngleDeg = currentAngle * (180 / PI);
    
    
    if(moveCommand == true){
          
          if ( currentAngleDeg - target_angle < 0 ) {
                     
            PWM[0] = PWM[0] - 1;
            PWM[1] = PWM[1] + 1;
      
            digitalWrite(dirPin[0], LOW);
            digitalWrite(dirPin[1], HIGH);
          }
          else if ( currentAngleDeg - target_angle > 0 ) {
            PWM[0] = PWM[0] + 1;
            PWM[1] = PWM[1] - 1;
      
            digitalWrite(dirPin[0], HIGH);
            digitalWrite(dirPin[1], LOW);
          }
        
          for (int i = 0; i < 2; i++) {
            analogWrite( pwmPin[i], max(0, PWM[i]) );
          }

          if( abs(currentAngleDeg - target_angle) <= 1 ){
            desired_pos[0] = 2 * PI * (float)(pos_counts[0]) / 3200;
            desired_pos[1] = 2 * PI * (float)(pos_counts[1]) / 3200;
            moveCommand = false;
          }

    }


    if(moveCommand == false) {
            Serial.println("static error correction");
            
            
        // Loops through calculations for each motor
          for (int i = 0; i < 2; i++) {
            // Positional and velocity calculations
            actual_pos[i] = 2 * PI * (float)pos_counts[i] / 3200;
            prev_actual_pos[i] = 2 * PI * (float)prev_counts[i] / 3200;
            delta_pos[i] = actual_pos[i] - prev_actual_pos[i];
            actual_speed[i] = delta_pos[i] / ((float)desired_Ts_ms / 1000);
            pos_error[i] = desired_pos[i] - actual_pos[i];
            integral_error[i] = integral_error[i] + pos_error[i] * ((float)desired_Ts_ms / 1000);
            desired_speed[i] = (P[i] * pos_error[i]) + (I[i] * integral_error[i]);
            error[i] = desired_speed[i] - actual_speed[i];
            Voltage[i] = Kp[i] * error[i];
        
            PWM[i] = 255 * abs(Voltage[i]) / Battery_Voltage;
        
            if ( pos_error[i] > 0 ) {
              digitalWrite(dirPin[i], HIGH);
            } else {
              digitalWrite(dirPin[i], LOW);
            }

            analogWrite( pwmPin[i], min(PWM[i], 50));
      
          }

    }
  
      //Serial print values with labels
      Serial.print("Time: ");
      Serial.print(current_time);
      Serial.print("\t xPos: ");
      Serial.print(currentXPos);
      Serial.print("\t yPos: ");
      Serial.print(currentYPos);
      Serial.print("\t phi: ");
      Serial.print(currentAngleDeg);
      Serial.print(" Desired Pos: ");
      Serial.print(desired_pos[0]);
      Serial.print(", ");
      Serial.print(desired_pos[1]);
      Serial.print(" Actual Pos: ");
      Serial.print(actual_pos[0]);
      Serial.print(", ");
      Serial.print(actual_pos[1]);
      Serial.print(" Pos Error: ");
      Serial.print(pos_error[0]);
      Serial.print(", ");
      Serial.print(pos_error[1]);
      Serial.print("\n");
  
    // Reset Values
    prev_counts[0] = pos_counts[0];
    prev_counts[1] = pos_counts[1];

    
  }

} // end void loop
