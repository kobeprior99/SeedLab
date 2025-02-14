// Pin definitions
const int enablePin = 4;    // Pin 4 to enable the motor driver
const int pwmPin1 = 9;      // Pin 9 for PWM control (Motor 1)
const int pwmPin2 = 10;     // Pin 10 for PWM control (Motor 2)
const int dirPin1 = 7;      // Pin 7 for direction control (Motor 1)
const int dirPin2 = 8;      // Pin 8 for direction control (Motor 2)

// Encoder pins
const int encPinA1 = 2; // Encoder A pin for motor 1
const int encPinB1 = 3; // Encoder B pin for motor 1
const int encPinA2 = 4; // Encoder A pin for motor 2
const int encPinB2 = 5; // Encoder B pin for motor 2

//only pins 2 and 3 have ISR functions, so update code for ISRs to make motor 1 and motor 2 use the pins and work together

// Encoder count variables
volatile long pos1_counts = 0; // Encoder position for motor 1
volatile long pos2_counts = 0; // Encoder position for motor 2

// Timing variables
unsigned long desired_Ts_ms = 10;  // Desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

// Interrupt service routines (ISR) for counting encoder pulses
void encoder1ISR_A() {
  if (digitalRead(encPinB1) == HIGH) {
    pos1_counts++; // Clockwise rotation
  } else {
    pos1_counts--; // Counter-clockwise rotation
  }
}

void encoder1ISR_B() {
  if (digitalRead(encPinA1) == HIGH) {
    pos1_counts--; // Counter-clockwise rotation
  } else {
    pos1_counts++; // Clockwise rotation
  }
}

void encoder2ISR_A() {
  if (digitalRead(encPinB2) == HIGH) {
    pos2_counts++; // Clockwise rotation
  } else {
    pos2_counts--; // Counter-clockwise rotation
  }
}

void encoder2ISR_B() {
  if (digitalRead(encPinA2) == HIGH) {
    pos2_counts--; // Counter-clockwise rotation
  } else {
    pos2_counts++; // Clockwise rotation
  }
}

void setup() {
  // Set motor control pins as outputs
  pinMode(enablePin, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  // Set encoder pins as inputs
  pinMode(encPinA1, INPUT);
  pinMode(encPinB1, INPUT);
  pinMode(encPinA2, INPUT);
  pinMode(encPinB2, INPUT);
  
  // Enable the motor driver by setting the enable pin HIGH
  digitalWrite(enablePin, HIGH);

  // Initialize Serial communication
  Serial.begin(115200); // Set baud rate to 115200 for fast data output

  // Initialize timing
  last_time_ms = millis();  // Record the start time for the first sample
  start_time_ms = last_time_ms;

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(encPinA1), encoder1ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinB1), encoder1ISR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinA2), encoder2ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinB2), encoder2ISR_B, CHANGE);
}

void loop() {
  // Convert encoder counts to radians
  float pos1_rad = 2 * PI * pos1_counts / 3200;
  float pos2_rad = 2 * PI * pos2_counts / 3200;

  // Calculate elapsed time in seconds
  current_time = (float)(last_time_ms - start_time_ms) / 1000.0;

  // Print the timestamp and positions in radians
  Serial.print(current_time);
  Serial.print("\t");
  Serial.print(pos1_rad);
  Serial.print("\t");
  Serial.print(pos2_rad);
  Serial.println("");

  // PWM control - Example with a fixed 128 PWM (50% duty cycle) for motors
  // higher number speeds up
  analogWrite(pwmPin1, 128); //motor 1
  analogWrite(pwmPin2, 128); //motor 2
  
  // Direction control - Motor 1 and Motor 2 forward
  digitalWrite(dirPin1, HIGH); // Motor 1 forward (HIGH is forward, LOW is backward)
  digitalWrite(dirPin2, LOW); // Motor 2 forward (LOW is forward, HIGH is backward)

  // Wait until the desired time has passed for the next sample
  while (millis() < last_time_ms + desired_Ts_ms) {
    // Wait for the next sample period
  }

  // Print results
  Serial.print("Motor 1 position (radians): ");
  Serial.print(pos1_rad);
  Serial.print("\n");
  Serial.print("Motor 2 position (radians): ");
  Serial.println(pos2_rad);
  Serial.print("\n");
  
  // Update last_time_ms to the current time for the next loop
  last_time_ms = millis();
}
