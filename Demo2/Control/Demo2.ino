/**
 * @file Demo2.ino
 * @brief SEED DEMO #2: Robot control system for sweeping, identifying, and driving towards a marker.
 *        The robot stops and reads within 1.5 feet, then either stays or turns 90 degrees based on instructions.
 *
 * @details
 * This program implements a finite state machine (FSM) to control a robot's behavior. The robot performs the following tasks:
 * - Sweeps to locate a marker using angle and distance data from a Raspberry Pi.
 * - Turns towards the marker.
 * - Drives towards the marker while maintaining control over its position and velocity.
 * - Stops at the marker and either stays or turns based on additional instructions.
 *
 * The program uses encoder feedback for motor control and communicates with a Raspberry Pi via I2C to receive instructions.
 *
 * @authors Cooper Hammond and Ron Gaines
 * @date Created on: March 24, 2025
 *
 * @hardware
 * - Motor driver with enable, PWM, and direction pins.
 * - Encoders for motor feedback.
 * - Raspberry Pi for external communication.
 *
 * @dependencies
 * - Wire.h: For I2C communication with the Raspberry Pi.
 *
 * @variables
 * - PI Communication:
 *   - angle_pi: Angle received from the Raspberry Pi (in degrees).
 *   - distance_pi: Distance received from the Raspberry Pi (in inches).
 *   - good_angle, good_distance: Flags indicating valid angle and distance data.
 *   - arrow: Directional command (0 = left, 1 = right, 2 = no arrow).
 * - Motor Control:
 *   - pos_counts: Encoder counts for motor positions.
 *   - motorVel: Motor velocities.
 *   - voltage: Motor voltages.
 * - Control Parameters:
 *   - kpPhi, kdPhi: Proportional and derivative gains for rotational control.
 *   - kpRho, kdRho: Proportional and derivative gains for linear control.
 * - FSM:
 *   - State: Current state of the robot (SWEEP, TURN, DRIVE, STOP).
 *   - atMarker: Flag indicating whether the robot is at the marker.
 *
 * @functions
 * - setup(): Initializes pins, I2C communication, and encoder interrupts.
 * - loop(): Main control loop implementing the FSM and control logic.
 * - encoder1_ISR(), encoder2_ISR(): Interrupt service routines for encoder pulse counting.
 * - receive(): Handles I2C communication to receive data from the Raspberry Pi.
 *
 * @notes
 * - The robot uses a proportional-derivative (PD) controller for both rotational and linear control.
 * - The program includes debug messages for state transitions and actions.
 * - Ensure the Raspberry Pi is booted before starting the robot.
 * 
 * Example Execution:
 * Place the robot's axis of rotation inline with an ArUco marker between 4 and 6 feet away. 
 * Ensure I2C connections are made as follows:
 * - Pi Pin 3 > Arduino A4
 * - Pi Pin 5 > Arduino A5
 * - Pi Pin 6 > Arduino GND
 * Turn on the Raspberry Pi and run the computer vision code. 
 * Turn on the robot, and the angle sweep will begin. The robot will then autonomously approach the target.
 */

//DEMO 2
//Cooper Hammond and Ron Gaines

// Control Booleans
bool doTurn = true;
bool atMarker = false;
bool correctiveAngleTurn = false;

// PI communication
boolean start = false;
#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;
const int BUFFER_SIZE = 4;
volatile float instruction_array[6]; // array to store the instructions

// PI recive Global Variables
byte buffer[BUFFER_SIZE];
volatile float angle_pi = 0.0;    // sent in degrees
volatile float distance_pi = 0.0; // sent in inches
volatile int good_angle = 0;      // 0=invalid , 1=valid
volatile int good_distance = 0;   // 0=invalid angle, 1=valid angle
volatile int arrow = 2;           // 0=left, 1=right, 2=no arrow

// Pin Definitions
const int enablePin = 4;      // Pin 4 to enable the motor driver
const int pwmPin[] = {9, 10}; // Pin 9 for motor 1, pin 10 for motor 2
const int dirPin[] = {7, 8};  // Pin 7 for motor 1, pin 8 for motor 2

// Encoder pins (one per motor)
const int encPin1A = 2; // Encoder pin A for motor 1
const int encPin2A = 3; // Encoder pin A for motor 2
const int encPin1B = 5; // Encoder pin B for motor 1
const int encPin2B = 6; // Encoder pin B for motor 2

// Timing variables
float desired_Ts_ms = 10; // Desired sample time in milliseconds
float last_time_ms;
float current_time_ms;

// Array variables [motor1, motor2] (mostly for mini project)
// volatile
volatile long int pos_counts[] = {0, 0};
volatile long int prev_counts[] = {0, 0};
// set in error correction
float pos_error[] = {0, 0};
// calculations
float desired_pos[] = {0, 0};     // in radians
float actual_pos[] = {0, 0};      // is pos_counts in radians
float prev_actual_pos[] = {0, 0}; // is prev_counts in radians

// Movement [motor1, motor2]
float PWM[] = {0, 0};
float motorVel[] = {0, 0};
float voltage[] = {0, 0};
float vBar = 0;
float deltaV = 0;

// Rotational control values
float currentPhi = 0.0;
float desiredPhi = 0.0; // use to set turn
float phiError = 0;
float prevPhiError = 0.0;
float dPhi = 0.0;
float kpPhi = 620;
float kdPhi = 45;
float angularVel = 0.0;
float desiredAngVel = 0;
float angularVelError = 0.0;

// Linear control values
float currentRho = 0.0;
float desiredRho = 0.0;
float rhoError = 0.0;
float prevRhoError = 0.0;
float dRho = 0.0;
float kpRho = 57;
float kdRho = 2.5;
float velocity = 0.0;
float desiredVel = 0.0;
float velocityError = 0.0;

// Constants/Physical Parameters
const float battery_voltage = 7.8;
const float b = 1;         // wheel base 12 inches
const float d = 5.93 / 12; // wheel diameter (inches)
const float r = d / 2;     // wheel radius (inches)

// FSM
enum State {
    SWEEP, TURN, DRIVE, STOP
};
State state = SWEEP; // initialize

void setup() {

    // PI communication
    Wire.begin(MY_ADDR);
    Wire.onReceive(receive);
    Wire.setClock(400000); // 400 kHz clock speed

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

    delay(1000); // delay for PI bootup
}

// Functions

// Interrupt service routines (ISR) for counting encoder pulses
void encoder1_ISR() {
    if (digitalRead(encPin1A) == digitalRead(encPin1B)){
        pos_counts[0] -= 2; // Clockwise rotation
    }
    else{
        pos_counts[0] += 2; // Counter-clockwise rotation
    }
}

void encoder2_ISR() {
    if (digitalRead(encPin2A) == digitalRead(encPin2B)){
        pos_counts[1] += 2; // Counter-clockwise rotation
    }
    else{
        pos_counts[1] -= 2; // Clockwise rotation
    }
}

void receive(int numBytes) {
    while (Wire.available()) {
        Wire.read(); // discard first byte (offset)
        good_angle = Wire.read();
        good_distance = Wire.read();
        arrow = Wire.read();
        // need to read four bytes and convert into float
        for (int i = 0; i < BUFFER_SIZE; i++) {
            buffer[i] = Wire.read();
        }
        memcpy(&angle_pi, buffer, sizeof(angle_pi));

        for (int i = 0; i < BUFFER_SIZE; i++) {
            buffer[i] = Wire.read();
        }
        memcpy(&distance_pi, buffer, sizeof(distance_pi));
    }
}

void loop() {

    // Find Current Time in miliseconds
    current_time_ms = millis();
    // find delta time in seconds
    float delta_t = ((float)(current_time_ms - last_time_ms)) / 1000; // should be about 0.01 seconds each loop
    last_time_ms = current_time_ms;

    // Turn Encoder Counts to Radians then find velocity
    for (int i = 0; i < 2; i++) {
        actual_pos[i] = 2 * PI * (float)(pos_counts[i]) / 3200;
        if (delta_t > 0) {
            motorVel[i] = (actual_pos[i] - prev_actual_pos[i]) / (delta_t); // (delta x) / (delta t) -> pos/s
        }
    }

    // Calculate Current Phi and Current Rho
    currentPhi = (r / b) * (actual_pos[1] - actual_pos[0]);
    currentRho = (r / 2) * (actual_pos[1] + actual_pos[0]);

    // FSM for robot actions SWEEP, TURN, DRIVE, STOP
    switch (state) {

        case SWEEP: // angle sweep
            if (good_angle != 1) {
                desiredPhi += 0.5 * PI / 180.0;
                Serial.println("Scanning for bitches...");
            }
            else if (good_angle == 1) {
                Serial.println("Found um");
                analogWrite(pwmPin[0], 0);
                analogWrite(pwmPin[1], 0);
                delay(500);
                desiredPhi = currentPhi + float(angle_pi * (PI / 180.0));
                //desiredRho = currentRho;
                state = TURN;
            }
            break;

        case TURN:          // turn to the phi we want
            desiredVel = 0; // we want to be stationary around axle center axis
            desiredRho = currentRho;
            Serial.println("Whip that thang around cuzo");

            // Check if we are within desired bounds
            if (fabs(currentPhi - desiredPhi) <= .015) {
                analogWrite(pwmPin[0], 0);
                analogWrite(pwmPin[1], 0);
                delay(1000);
                desiredPhi = currentPhi;
                desiredRho = currentRho + float(distance_pi / 12.0) + 0.2;
                if (atMarker == true) {
                    atMarker = false;
                    state = STOP;
                }
                else if (correctiveAngleTurn == false){
                  delay(500);
                  desiredPhi = currentPhi + float(angle_pi * (PI / 180.0));
                  correctiveAngleTurn = true;
                  state = TURN;
                }
                else {
                    state = DRIVE;
                }
            }
            break;

        case DRIVE:       // drive to the rho we want
            kdPhi = 4.83; // 14.0
            Serial.println("They see me rolling");
            if ((currentRho - (desiredRho - 1.75)) <= 1.0 && (currentRho - (desiredRho - 1.75)) >= 0) {
                atMarker = true;
                state = STOP;
            }
            break;

        case STOP: // stop and stay where you are
            analogWrite(pwmPin[0], 0);
            analogWrite(pwmPin[1], 0);
            kdPhi = 45;
            if (atMarker == true && doTurn == true) {
                if (arrow == 0) { // left
                    Serial.println("To the left, to the left, to the left");
                    delay(2000);
                    desiredPhi = currentPhi + (PI / 2.0);
                    state = TURN;
                }
                else if (arrow == 1) { // right
                    Serial.println("To the right, to the right, to the right");
                    delay(2000);
                    desiredPhi = currentPhi - (PI / 2.0);
                    state = TURN;
                }
                else { // no turn comand from pi
                    Serial.println("Now Freeze!");
                    analogWrite(pwmPin[0], 0);
                    analogWrite(pwmPin[1], 0);
                    break;
                }
            }
            else {
                Serial.println("Unlike my wife, I have finished.");
                analogWrite(pwmPin[0], 0);
                analogWrite(pwmPin[1], 0);
            }

            break;
        }

    // Controler Logic
    if (state == TURN || state == DRIVE || state == SWEEP) {
        // Rotational Controller (Phi)
        phiError = desiredPhi - currentPhi;
        dPhi = (phiError - prevPhiError) / delta_t;
        // iPhi += phiError * delta_t;
        desiredAngVel = (kpPhi * phiError) + (kdPhi * dPhi);
        angularVel = (r / b) * (motorVel[1] - motorVel[0]);

        angularVelError = desiredAngVel - angularVel;
        rhoError = desiredRho - currentRho;
        dRho = (rhoError - prevRhoError) / delta_t;
        desiredVel = (kpRho * rhoError) + (kdRho * dRho);
        velocity = (r / 2) * (motorVel[1] + motorVel[0]);

        velocityError = desiredVel - velocity;
        vBar = velocityError;
        deltaV = angularVelError;

        // bound controller instead of PWM
        if (abs(vBar) >= battery_voltage) {
            vBar = battery_voltage * (vBar / abs(vBar)); // keep the sign
        }

        if (abs(deltaV) >= battery_voltage) {
            deltaV = battery_voltage * (deltaV / abs(deltaV)); // keep the sign
        }

        // Use vBar and deltaV to find motor voltages
        voltage[0] = (vBar - deltaV) / 2;
        voltage[1] = (vBar + deltaV) / 2;

        // Check voltage sign to give motors directions (fwd/bkwd) then send PWM signals
        for (int i = 0; i < 2; i++) {
            // if voltage pos direction fwd
            if (voltage[i] > 0) {
                digitalWrite(dirPin[i], HIGH);
            }
            // if voltage neg direction bkwd
            else {
                digitalWrite(dirPin[i], LOW);
            }
            // get pwm based on battery voltage
            PWM[i] = 255 * (abs(voltage[i]) / battery_voltage);
            analogWrite(pwmPin[i], PWM[i]);
        }

        // Update Values
        prevPhiError = phiError;
        prevRhoError = rhoError;
        for (int i = 0; i < 2; i++) {
            prev_counts[i] = pos_counts[i];
            prev_actual_pos[i] = actual_pos[i];
        }
        
        while (millis() < last_time_ms + desired_Ts_ms) {
            // wait
        }
    }
}