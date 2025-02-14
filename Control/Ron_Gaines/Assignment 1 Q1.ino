//This code outputs PWM signals to four LEDs to turn them on and off in a sequence. The rate this happens at can be controlled using 
//the gas and break buttons which are given interrupts.
//SEED: Assignment #1
//Ronald Gaines

//assign pins to names or groups
int gas = 2;
int brake = 3;
int ledPins[] = {5, 6, 9, 10};

//set bounds and initial settings for pattern movement
int delayTime = 500;
int minDelay = 100;  
int maxDelay = 1000;    
int stepDelay = 100;

void setup() {
  //set LED pints to output
  pinMode(ledPins[0], OUTPUT);
  pinMode(ledPins[1], OUTPUT);
  pinMode(ledPins[2], OUTPUT);
  pinMode(ledPins[3], OUTPUT);

  //set gas and brake pins to input
  pinMode(gas, INPUT);
  pinMode(brake, INPUT);
 
  //set hardware interupts to gas and break pins
  attachInterrupt(digitalPinToInterrupt(gas), increaseSpeed, FALLING);
  attachInterrupt(digitalPinToInterrupt(brake), decreaseSpeed, FALLING);
}

//LED pattern
void loop() {
  //Turn LEDs off in forward sequence
  for (int i = 0; i < 4; i++) {
    digitalWrite(ledPins[i], HIGH);
    delay(delayTime);
    digitalWrite(ledPins[i], LOW);
  }

  // Turn LEDs off in reverse sequence
  for (int i = 3; i >= 0; i--) {
    digitalWrite(ledPins[i], HIGH);
    delay(delayTime);
    digitalWrite(ledPins[i], LOW);
  }
}

//set upper bound
void increaseSpeed() {
  if (delayTime > minDelay) {
    delayTime -= stepDelay;
  }
}

//set lower bound
void decreaseSpeed() {
  if (delayTime < maxDelay) {
    delayTime += stepDelay;
  }
}
