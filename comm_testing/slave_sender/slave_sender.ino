/**
 * To be uploaded on motor driver module boards
 */

// NANO -> ATMEGA328-AU Board Pin Mapping
//  Nano dec port   ATMega328-AU            Nano dec port       ATMega328-AU
//  -------------   --------------------    -------------    -------------------
//  pin (12) D12 -> IC pin 16 (MISO)        pin (13) D13  -> IC pin 17 (SCK)
//  pin (11) D11 -> IC pin 15 (MOSI)        pin 3.3V
//  pin (10) D10 -> IC pin 14 (PB2)         pin AREF -> IC pin 20 (AREF)
//  pin ( 9) D09 -> IC pin 13 (PB1)         pin (14) PC0  -> IC pin 23 (PC0/ADC0)
//  pin ( 8) D08 -> IC pin 12 (PB0)         pin (15) PC1  -> IC pin 24 (PC1/ADC1)
//  pin ( 7) D07 -> IC pin 11 (PD7)         pin (16) PC2  -> IC pin 25 (PC2/ADC2)
//  pin ( 6) D06 -> IC pin 10 (PD6)         pin (17) PC3  -> IC pin 26 (PC3/ADC3)
//  pin ( 5) D05 -> IC pin 09 (PD5)         pin (18) PC4  -> IC pin 27 (PC4/ADC4)
//  pin ( 4) D04 -> IC pin 02 (PD4)         pin (19) PC5  -> IC pin 28 (PC5/ADC5)
//  pin ( 3) D03 -> IC pin 01 (PD3)         pin (??) ADC6 -> IC pin 19 (ADC6)
//  pin ( 2) D02 -> IC pin 32 (PD2/INT0)    pin (??) ADC7 -> IC pin 22 (ADC7)
//  pin (??) RST -> IC pin 29 (PC6/RESET)   pin (??) PC6  -> IC pin 29 (RESET/PC6)
//  pin (??) RXD -> IC pin 30 (RX/PD0)      pin GND
//  pin (??) TXD -> IC pin 31 (TX/PD1)      pin VIN
//  --- --- -> IC pin 08 (PB7/OSC2)    --- ---  -> IC pin 07 (PB8/OSC1)
//  --- --- -> IC pin 03/05/21 (GND)   --- ---  -> IC pin 04/06/18 (VCC/VCC/AVCC)


#include <Wire.h>

#define NSLEEP 5
#define INPUT1 9
#define INPUT2 10

String shape_sequence = "1, 1, 1, 4";  //test string
bool I2CFlag = false; // motor pwm on i2c request flag

void setup() {
  //Serial.begin(9600); //begin Serial - lower baud rates work better for AtMega328 board
  
  //set up I2C address as 0x01 for the current board - in the future this will be sequential for all boards so the master can address them individually
  Wire.begin(0x01); 
  
  //upon receiving a request from the master, call requestEvent
 // Wire.onRequest(requestEvent); 
}

void loop() {
  //printString( "hello");


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


//  /***
//   * Motor operation code
//   */
//   digitalWrite(NSLEEP, HIGH); //nSleep should be kept high for normal operation
//
//   //turn motor off
//   if(I2CFlag) {
//   analogWrite(INPUT1, 0);
//   analogWrite(INPUT2, 0);
//   I2CFlag = false;
//   }
//
//   //turn motor on
//   else { 
//   analogWrite(INPUT1, 0);
//   analogWrite(INPUT2, 255);
//   I2CFlag = true;
//   }
   
}

void motorTest() {
  
}
