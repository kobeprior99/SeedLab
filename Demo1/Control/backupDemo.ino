// SEED DEMO #1 
// Implemented with a PID Controller, Final Version
// Localization and Controls
// Ron Gaines and Cooper Hammond 

boolean start = false;
#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;
// instruction[0] = distance, instruction[1] = angle
const int BUFFER_SIZE = 24; //6 floats*4 bytes each
volatile float instruction_array[6]; //array to store the instructions



const float instructionDegFt[] = {0, 2}; //{angle in degrees, distance in feet
const float instruction[] = {(instructionDegFt[0] * (PI/180)), instructionDegFt[1]};

// Pin Definitions
const int enablePin = 4;       // Pin 4 to enable the motor driver
const int pwmPin[] = {9, 10};  // Pin 9 for motor 1, pin 10 for motor 2
const int dirPin[] = {7, 8};   // Pin 7 for motor 1, pin 8 for motor 2

// Encoder pins (one per motor)
const int encPin1A = 2; // Encoder pin A for motor 1
const int encPin2A = 3; // Encoder pin A for motor 2
const int encPin1B = 5; // Encoder pin B for motor 1
const int encPin2B = 6; // Encoder pin B for motor 2

// Timing variables
float desired_Ts_ms = 10;  // Desired sample time in milliseconds
float last_time_ms;
float start_time_ms;
float current_time;

// Array variables [motor1, motor2] (mostly for mini project)
  //volatile
volatile int pos_counts[] = {0, 0};
volatile int prev_counts[] = {0, 0};
  //set in error correction
float pos_error[] = {0, 0};
float integral_error[] = {0, 0};
float desired_speed[] = {0, 0};
float error[] = {0, 0};
  //calculations
float desired_pos[] = {0, 0};     // in radians
float actual_pos[] = {0, 0};      // is pos_counts in radians
float prev_actual_pos[] = {0, 0}; // is prev_counts in radians
float delta_pos[] = {0, 0};
float actual_speed[] = {0, 0};

// Movement [motor1, motor2]
float PWM[] = {0, 0};
float motorVel[] = {0, 0};
float voltage[] = {0, 0};
float vBar = 0;
float deltaV = 0;
  // Angular Controller
float currentPhi = 0;
float desiredPhi = instruction[0];   // use to set turn
float phiError = 0;
float prevPhiError = 0.0;
float dPhi = 0.0;
float iPhi = 0.0;
float kpPhi = 3500; 
float kdPhi = 500;
float kiPhi = 5; 
float angularVel = 0.0;
float desiredAngVel = 0;
float angularVelError = 0.0;
float kpAngVel = 1;
  // Driving Controller
float currentRho = 0.0;
float desiredRho = 0.0;   // use to set distance
float rhoError = 0.0;
float prevRhoError = 0.0;
float dRho = 0.0;
float iRho = 0.0;
float kpRho = 13; 
float kdRho = 1.5; 
float kiRho = 0.25;  
float velocity = 0.0;
float desiredVel = 0.0;
float velocityError = 0.0;
float kpVel = 4;    

// Controller Constants [motor1, motor2]
const float K[] = {1.5, 1.75};
const float sigma[] = {14, 14};
const float Kp[] = {1.75, 2};
const float P[] = {4.2, 4};
const float I[] = {0.1, 0.1};

// Constants/Physical Parameters
const float battery_voltage = 7.8;
const float b = 1;         // wheel base 12inches
const float d = 5.93 / 12;  // wheel diameter (feet)
const float r = d / 2;      // wheel radius (feet)

// Position initialized (from 2b localization code)
float currentXPox = 0.0;
float lastXPos = 0.0;
float currentYPos = 0.0;
float lastYPos = 0.0;
float currentAngle = 0.0;
float lastAngle = 0.0;
float currentPos = 0.0;


const float maxAcceleration = 0.5;
const float maxPWMChange = 10;


// FSM
enum State { TURN, DRIVE, STOP };  // three states
State state = TURN;                 // initialize

void setup() {

  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);
  // Initialize Serial communication
  Serial.begin(115200); // Set baud rate to 115200 for fast data output
 
  // Set motor control pins as outputs
  pinMode(enablePin, OUTPUT);
  pinMode(pwmPin[0], OUTPUT);
  pinMode(pwmPin[1], OUTPUT);
  pinMode(dirPin[0], OUTPUT);
  pinMode(dirPin[1], OUTPUT);

  // Set encoder pins as inputs
  pinMode(encPin1A, INPUT);
  pinMode(encPin2A, INPUT);
  pinMode(encPin1B, INPUT);
  pinMode(encPin2B, INPUT);
  // Enable the motor driver by setting the enable pin HIGH
  digitalWrite(enablePin, HIGH);

  // Initialize timing
  last_time_ms = millis();  // Record the start time for the first sample
  start_time_ms = last_time_ms;

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(encPin1A), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPin2A), encoder2_ISR, CHANGE);
}


void receive(int numBytes){
  
    if (numBytes == BUFFER_SIZE + 1){
      Wire.read(); //discard first byte (offset)
      byte buffer[BUFFER_SIZE];
      Wire.readBytes(buffer, BUFFER_SIZE);
      memcpy(instruction_array, buffer, BUFFER_SIZE);
      desiredPhi = instruction_array[1]*(PI/180);   // use to set turn

      //debug:
      // float good_angle = instruction_array[0];
      // float angle = instruction_array[1];
      // float good_distance = instruction_array[2];
      // float distance = instruction_array[3];
      // float good_arrow = instruction_array[4];
      // float arrow = instruction_array[5];
      // Serial.println("Received instructions:");
      // Serial.print("Angle: ");
      // if(good_angle == 1.0) Serial.println(angle);
      // else Serial.println("N/A");
      // Serial.print("Distance: ");
      // if(good_distance == 1.0) Serial.println(distance);
      // else Serial.println("N/A");
      // Serial.print("Arrow: ");
      // if(good_arrow == 1.0) Serial.println(arrow);
      // else Serial.println("N/A");
      // Serial.println("start == true");

      //start the rest of the code
      start = true;
    }

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
  while(!start){
    delay(100);
  }
  // Find Current Time
  current_time = (float)( millis() - start_time_ms ) / 1000;

  // Turn Encoder Counts to Radians then find velocity
  for ( int i = 0; i < 2; i++ ) {
    actual_pos[i] = 2 * PI * (float)( pos_counts[i]) / 3200;
    if( current_time > 0 ) {
      motorVel[i] = ( actual_pos[i] - prev_actual_pos[i] ) / current_time;
    }
  }
  
  // Calculate Current Phi and Current Rho
  currentPhi = ( r / b ) * ( actual_pos[1] - actual_pos[0] );
  currentRho = ( r / 2 ) * ( actual_pos[1] + actual_pos[0] );

  // FSM for robot actions
  switch ( state ) {
    case TURN: // turn to the phi we want
      desiredVel = 0;    // we want to be stationary
      desiredRho = 0;

      // Check if we are within desired bounds
      if ( fabs(currentPhi-desiredPhi) <= 0.02 ) {
        analogWrite( pwmPin[0], 0);
        analogWrite( pwmPin[1], 0);
        delay(1000);
        desiredPhi = currentPhi;
//        digitalWrite( dirPin[0], HIGH );
//        digitalWrite( dirPin[1], HIGH );
//        analogWrite( pwmPin[0], 80);
//        analogWrite( pwmPin[1], 56);
//        delay(500);
        state = DRIVE;
      }
      break;
      
    case DRIVE: // drive to the rho we want
      desiredRho = instruction_array[3];
      kdPhi = 60.0;  

        
      if(fabs(currentRho-desiredRho)< 0.02) {
        state = STOP;
      }
      break;
    
    case STOP: // stop and stay where you are
      desiredPhi = currentPhi;
      desiredRho = currentRho;
      analogWrite( pwmPin[0], 0);
      analogWrite( pwmPin[1], 0);

      break;
  }

  // Controler Logic
  if ( state == TURN || state == DRIVE ) {
    // Rotational Controller (Phi)
    phiError = desiredPhi - currentPhi;
    dPhi = ( phiError - prevPhiError ) / ((float)( desired_Ts_ms / 1000 ));
    iPhi += phiError * ((float)( desired_Ts_ms / 1000 ));
    desiredAngVel = ( kpPhi * phiError ) + ( kdPhi * dPhi ) + ( kiPhi * iPhi );
    angularVel = ( r / b ) * ( motorVel[1] - motorVel[0] );
    angularVelError = desiredAngVel - angularVel;

    // Driving Controller (Rho)
    rhoError = desiredRho - currentRho;
    dRho = ( rhoError - prevRhoError ) / ((float)( desired_Ts_ms / 1000 ));
    iRho += rhoError * ((float)( desired_Ts_ms / 1000 ));
    iRho =constrain(iRho, -4, 4);
    if(fabs(rhoError)< 0.02){
      iRho = 0; //unwind integral term when close enough
    }
    desiredVel = ( kpRho * rhoError ) + ( kdRho * dRho ) + ( kiRho * iRho );
    velocity = ( r / 2 ) * ( motorVel[1] + motorVel[0] );
    velocityError = desiredVel - velocity;

    // Calculate vBar and deltaV
    vBar = velocityError * kpVel;;
    deltaV = angularVelError * kpAngVel;

    // Use vBar and deltaV to find motor voltages
    voltage[0] = ( vBar - deltaV ) / 2;
    voltage[1] = ( vBar + deltaV ) / 2;

    // Check voltage sign to give motors directions (fwd/bkwd) then send PWM signals
    for ( int i = 0; i < 2; i++ ) {
      if ( voltage[i] > 0 ) {
        digitalWrite( dirPin[i], HIGH );
      }
      else {
        digitalWrite( dirPin[i], LOW );
      }

      PWM[i] = 255 * (abs( voltage[i] ) / battery_voltage);
      analogWrite( pwmPin[i], min( PWM[i], 150 ) );
    }
  } 
  Serial.print("Time: ");
  Serial.print(current_time);
  Serial.print("Rho: ");
  Serial.print(currentRho);
  Serial.print(" Rho Error: ");
  Serial.print(rhoError);
  Serial.print(" Phi: ");
  Serial.print(currentPhi);
  Serial.print(" Phi Error: ");
  Serial.print(phiError);
  Serial.print(" angVel: ");
  Serial.print(angularVel);
  Serial.print(" State: ");
  Serial.println(state);
    
  // Update Values
  prevPhiError = phiError;
  prevRhoError = rhoError;

  for ( int i = 0; i < 2; i++ ) {
    prev_counts[i] = pos_counts[i];
    prev_actual_pos[i] = actual_pos[i];
}

  //start_time_ms = millis();
  while( millis() < last_time_ms + desired_Ts_ms ) {
    // wait
  }
  last_time_ms = millis();
}