/**
 * To be uploaded on motor driver module boards
 */


#include <Wire.h>
String shape_sequence = "1, 1, 1, 4";  //test string

void setup() {
  Serial.begin(9600); //begin Serial - lower baud rates work better for AtMega328 board
  
  //set up I2C address as 0x01 for the current board - in the future this will be sequential for all boards so the master can address them individually
  Wire.begin(0x01); 
  
  //upon receiving a request from the master, call requestEvent
  Wire.onRequest(requestEvent); 
}

void loop() {
  //printString( "hello");
  delay(1000);

}

char buffer[4];
/**
 * A function to take in the desired String message and write it to Serial as an array of characters
 * SAMI Board sends Serial message to Arduino UNO which translates it using the FTDI chip
 */
void printString(String message) {
  //convert input message to array of characters by manually separating characters
  char buffer[message.length() + 1];
  for (int i = 0; i < message.length(); i++) {
    buffer[i] = message.charAt(i);
  }

  //add null terminator at the end to tell Serial that transmission is over
  buffer[message.length()]= '\0';
  Serial.write(buffer);


}

/**
 * Callback function upon receiving a request via I2C from master
 */
void requestEvent() {
  //char buffer[shape_sequence.length()]; 
  // shape_sequence.toCharArray(buffer, shape_sequence.length());

  //generate output text char array
  buffer[0] = 'p';
  buffer[1] = 'o';
  buffer[2] = 'o';
  buffer[3] = '\0';

  //Write via I2C
  Wire.write(buffer);

  //print to Serial and insert a new line after transmission is complete
  // Serial.write(buffer);
  printString("hello");
  Serial.println(buffer);
  Serial.write('\n');
}
