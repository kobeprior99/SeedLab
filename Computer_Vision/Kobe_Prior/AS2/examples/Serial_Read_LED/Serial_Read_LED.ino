//code that reads serial and turns led on if 2 and turns off if 0
int incomingByte = 0;
int led=2;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(led, OUTPUT);
}11

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
    incomingByte = Serial.read();
    Serial.print("I recieved: ");
    Serial.println(char(incomingByte));
    if(incomingByte == '1'){
      digitalWrite(led, HIGH);
      }else{
       digitalWrite(led, LOW);
        }
    }
}
