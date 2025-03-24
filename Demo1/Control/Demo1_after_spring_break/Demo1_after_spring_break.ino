//Modified backup Demo
#include <Wire.h>
boolean start = false;

#define MY_ADDR 8
volatile uint8_t offset = 0;
// instruction[0] = distance, instruction[1] = angle
const int BUFFER_SIZE = 24; //6 floats*4 bytes each
volatile float instruction_array[6]; //array to store the instructions


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
long last_time_ms = 0;
long current_time_ms;

// Array variables [motor1, motor2] (mostly for mini project)
long int pos_counts[] = {0, 0};
long int prev_counts[] = {0, 0};
//set in error correction
float pos_error[] = {0, 0};
//calculations
float desired_pos[] = {0, 0};     // in radians
float actual_pos[] = {0, 0};      // is pos_counts in radians
float prev_actual_pos[] = {0, 0}; // is prev_counts in radians

// Movement [motor1, motor2]
float PWM[] = {0, 0};
float motorVel[] = {0, 0};
float voltage[] = {0, 0};
float vBar = 0;
float deltaV = 0;
float currentPhi = 0;
float desiredPhi;   // use to set turn
float phiError = 0;
float prevPhiError = 0.0;
float dPhi = 0.0;
float iPhi = 0.0;
float kpPhi = 400; 
float kdPhi = 70;
float kiPhi = 3; 
float angularVel = 0.0;
float desiredAngVel = 0;
float angularVelError = 0.0;
float kpAngVel = 0.5;
  // Driving Controller
float currentRho = 0.0;
float desiredRho = 0.0;
float rhoError = 0.0;
float prevRhoError = 0.0;
float dRho = 0.0;
float iRho = 0.0;
float kpRho = 18; 
float kdRho = 1; 
float kiRho = 0.5;  
float velocity = 0.0;
float desiredVel = 0.0;
float velocityError = 0.0;
float kpVel = 1; 

//filter velocities
const float alpha = 0.4;   // Smoothing factor for velocity
const float alpha_ang = 0.4;  // Smoothing factor for angular velocity

// Constants/Physical Parameters
const float battery_voltage = 7.8;
const float b = 1;         // wheel base 12 inches 1 foot
const float d = 5.93 / 12;  // wheel diameter (feet)
const float r = d / 2;      // wheel radius (feet)

// FSM
enum State { TURN, DRIVE, STOP };  // three states
State state = TURN;                // initialize

void setup() {
  //i2c communication
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
      desiredPhi = instruction_array[1] * (PI/180);   // use to set turn in radians
      desiredRho = instruction_array[3];
      //start the rest of the code
      start = true;
    }
  }


// Functions

// Interrupt service routines (ISR) for counting encoder pulses
void encoder1_ISR() {
  //code to spend less time in isr and increments as before
  pos_counts[0] += (digitalRead(encPin1A) == digitalRead(encPin1B)) ? -2 : 2;
}

void encoder2_ISR() {
  //code to spend less time in isr and increments as before
  pos_counts[1] += (digitalRead(encPin2A) == digitalRead(encPin2B)) ? 2 : -2;
}

void loop() {
  //wait until data received from pi
  while(!start){
    delay(100);
  }
  // Find Current Time in miliseconds
  current_time_ms = millis();
  //find delta time in seconds
  float delta_t = ((float)(current_time_ms - last_time_ms)) / 1000; //should be about 0.01 seconds each loop
  last_time_ms = current_time_ms;

  // Turn Encoder Counts to Radians then find velocity
  for ( int i = 0; i < 2; i++ ) {
    actual_pos[i] = 2 * PI * (float)( pos_counts[i]) / 3200;
    if( delta_t > 0 ) {
      motorVel[i] = ( actual_pos[i] - prev_actual_pos[i] ) / (delta_t); // (delta x)/(delta t) -> pos/s
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
      if ( fabs(currentPhi - desiredPhi) <= 0.01 ) {
        analogWrite( pwmPin[0], 0);
        analogWrite( pwmPin[1], 0);
        delay(1000);
        desiredPhi = currentPhi;
        state = DRIVE;
      }
      break;
      
    case DRIVE: // drive to the rho we want
      desiredRho = instruction_array[3];
      //kdPhi = 14;
      kiPhi = 15;
      //check that rho and phi are within reasonable error
      if(fabs(currentRho - desiredRho) < 0.02 && fabs(currentPhi - desiredPhi) < 0.01) {
        state = STOP;
      }
      break;
    
    case STOP: // stop and stay where you are
      analogWrite( pwmPin[0], 0);
      analogWrite(pwmPin[1], 0);
      break;
  }

  // Controler Logic
  if(state == TURN || state == DRIVE){
  // Rotational Controller (Phi)
  phiError = desiredPhi - currentPhi;
  dPhi = ( phiError - prevPhiError ) / delta_t;
  iPhi += phiError * delta_t;
  desiredAngVel = ( kpPhi * phiError ) + (kiPhi * iPhi) + (kdPhi * dPhi);
  angularVel = ( r / b ) * ( motorVel[1] - motorVel[0] );
  // filteredAngularVel = alpha_ang * angularVel + (1-alpha_ang) * filteredAngularVel;
  //debug serial plot to see what filtered angular velocity looks like
  // Serial.print(filteredAngularVel);
  // Serial.print(angularVel);
  angularVelError = desiredAngVel - angularVel;


  // Driving Controller (Rho)
  rhoError = desiredRho - currentRho;
  dRho = ( rhoError - prevRhoError ) / delta_t;
  iRho += rhoError * delta_t;
  iRho = constrain(iRho, -4, 4);
  desiredVel = ( kpRho * rhoError ) + (kiRho * iRho) + (kdRho * dRho);
  velocity = ( r / 2 ) * ( motorVel[1] + motorVel[0] );
  //IIR filter from ISS2
  // filteredVelocity = alpha * velocity +(1 - alpha) * filteredVelocity;
  //debug serial plot to see what filtered velocity looks like
  // Serial.print(filteredVelocity);
  // Serial.print(velocity);
  velocityError = desiredVel - velocity;

  // Calculate vBar and deltaV
  vBar = velocityError * kpVel;
  deltaV = angularVelError * kpAngVel;
  // Use vBar and deltaV to find motor voltages
  voltage[0] = (vBar - deltaV) / 2;
  voltage[1] = (vBar + deltaV ) / 2;

  // Check voltage sign to give motors directions (fwd/bkwd) then send PWM signals
  for ( int i = 0; i < 2; i++ ) {
    //if voltage pos direction fwd
    if ( voltage[i] > 0 ) {
      digitalWrite( dirPin[i], HIGH );
    }
    //if voltage neg direction bkwd
    else {
      digitalWrite( dirPin[i], LOW );
    }
    //get pwm based on battery voltage
    PWM[i] = 255 * (abs( voltage[i] ) / battery_voltage);
    //debug: the voltage getting supplied to each motor
    // Serial.print("voltage");
    // Serial.print(i);
    // Serial.print(": ");
    // Serial.println(abs(voltage[i]));

    //note when voltage is greater than battery_voltage there is saturation and pwm gets capped, this causes motor 1 to fall behind
    //what is the fix: maybe if statement if i = 0 then saturation pwm is 110 instead of 100? so that motor has more power during saturation. Fine tune as needed
    if (i == 0){
      analogWrite( pwmPin[i], min( PWM[i], 128 ) );//caps pwm at 110
    }
    else{
    analogWrite( pwmPin[i], min( PWM[i], 125 ) );//caps pwm at 120
  }
  }
  
  //debug: print out all the values note state 0 = turn, 1 = drive, 2 = stop
  Serial.print("Time: ");
  Serial.print(current_time_ms);
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

  //consistant sampling rate 10 ms ideally
  while( millis() < last_time_ms + desired_Ts_ms ) {
    // wait to get close to desired sample rate
  }
}
}