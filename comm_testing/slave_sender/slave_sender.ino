#include <Wire.h>
String shape_sequence = "1, 1, 1, 4";

void setup() {
  Serial.begin(9600);
  Wire.begin(0x01);
  Wire.onRequest(requestEvent);

}

void loop() {
  printString( "hello");
  delay(1000);

}

char buffer[4];

void printString(String message) {
  char buffer[message.length() + 1];
  for (int i = 0; i < message.length(); i++) {
    buffer[i] = message.charAt(i);
  }
  buffer[message.length()]= '\0';
  Serial.write(buffer);


}
void requestEvent() {
  //char buffer[shape_sequence.length()];
  // shape_sequence.toCharArray(buffer, shape_sequence.length());
  buffer[0] = 'p';
  buffer[1] = 'o';
  buffer[2] = 'o';
  buffer[3] = '\0';
  Wire.write(buffer);
  // Serial.write(buffer);
  Serial.println(buffer);
  Serial.write('\n');
}
