/**
 * Uploaded to Arduino UNO - test code to receive Serial data from SAMI board
 */


int incomingByte = 0; // for incoming serial data - set up as null to start
void setup() {
  Serial.begin(9600); 

}

void loop() {
  //While there is data along the Serial bus (Serial.available() returns # of bytes along bus)
  while (Serial.available()) {
    // read the incoming byte:
    incomingByte = Serial.read();

    //convert ASCII value to readable character in Serial monitor
    Serial.print(char(incomingByte));
  }

}
