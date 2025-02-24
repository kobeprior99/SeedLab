#include <Wire.h>
#define MY_ADDR 8
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t reply = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin(MY_ADDR)
  Wire.onReceive(receive);
  Wire.onRequest(request);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(msgLength > 0){
    if(offset == 1){
      digitalWrite(LED_BUILTIN, instruction[0]);
      }
      printRecieved();
      msgLength = 0;
    }
}
void printRecieved(){
  Serial.print("Offset recieved: ");
  Serial.println(offset);
  Serial.print("message Length: ");
  Serial.println(msgLength);
  Serial.print("Instruction recieved: ");
  for (int i = 0; i<msgLength;i++){
    Serial.print(String(instruction[i])+"\t");
    }
    Serial.println("");
  }

 void receive(){
  offset = Wire.read();
  while(Wire.available()){
    instruction[msgLength] = Wire.read();
    msgLength++;
    reply = msgLength;
    }
  }

  void request(){
    Wire.write(reply);
    reply = 0;
    }
