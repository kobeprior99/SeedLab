void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial){
    ;
    }
    Serial.println("ASCII Table Character MAP");
}
int thisByte = 33;
void loop() {
  // put your main code here, to run repeatedly:
  Serial.write(thisByte);
  Serial.print(", dec: ");
  Serial.print(thisByte);


 Serial.print(", hex: ");
 Serial.print(thisByte, HEX);

 Serial.print(", oct: ");
 Serial.print(thisByte, OCT);

 Serial.print(", bin: ");
 Serial.println(thisByte, BIN);

 if (thisByte == 126){
  while(true){
    continue;
    }
  }
  thisByte++;
  
}
