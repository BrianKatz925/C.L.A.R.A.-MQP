int incomingByte = 0; // for incoming serial data
void setup() {
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available()) {
    // read the incoming byte:
    incomingByte = Serial.read();
  
    Serial.print(char(incomingByte));
  }

}
