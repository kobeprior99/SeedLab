
#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
void setup() {
  // put your setup code here, to run once:
  //baud rate
  Serial.begin(115200);
  //address
  Wire.begin(MY_ADDR);
  //when receiving call reeceive function
  Wire.onReceive(receive);
 
}

void loop() {
  // put your main code here, to run repeatedly:
  if (msgLength > 0){
    printRecieved();
    msgLength = 0;
    }
}
//function to print the string and ascii for each character to serial monitor
void printRecieved(){
  Serial.println("String recieved: ");
  for (int i = 0; i< msgLength; i++){
    //type cast char to get character version of ascii
    Serial.print(String(char(instruction[i])));
  }
  Serial.println("");//newline
  Serial.println("ASCII Codes");
  for (int i = 0; i < msgLength; i++){
    //note instructions are sent in ascii so we just need to type cast string for serial print
    Serial.print(String(instruction[i])+"");
    }
    Serial.println("");//newline
  
  }

void receive(){
  offset = Wire.read();//even though we don't use the offset you have to read this data
  while(Wire.available()){
    //populate instructions (characters)
    instruction[msgLength] = Wire.read();
    msgLength++;
    }
  }
