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
#include <math.h>

#define NSLEEP 5
#define INPUT1 9
#define INPUT2 10
#define currentRead 1 //replace
#define enc1 2 //replace
#define enc2 3 //replace 

int I2Cstatus = 0;
int slow = 100; //default slow speed 
int fast = 255; //default fast speed
int count = 0; //for encoder count

bool I2CFlag = false; // motor pwm on i2c request flag

void setup() {
  Serial.begin(9600); //begin Serial - lower baud rates work better for AtMega328 board
  
  //set up I2C address as 0x01 for the current board - in the future this will be sequential for all boards so the master can address them individually
   Wire.begin(0x01); 
  
  //upon receiving a request from the master, call requestEvent
 // Wire.onRequest(requestEvent); 
    Wire.onReceive(msgEvent);
}

void loop() {
  //main state machine to call functions based on what's being recieved on I2C
  if (I2CFlag){ //we have a new message
    switch(I2Cstatus){
      default: //no message- status defaults to zero
        break;
      case 1: //motor stop
        brake();
        I2CFlag = false; //since it's complete
        break;
      case 2: //motor forward slowly
        forward(slow);
        I2CFlag = false;
        break;
      case 3: //motor forward fast
        forward(fast);
        I2CFlag = false;
        break;
      case 4: //motor reverse slowly
        reverse(slow);
        I2CFlag = false;
        break;
      case 5: //motor reverse fast
        reverse(fast);
        I2CFlag = false;
        break;
      case 6: //get encoder count from motors
        count = getEncCount();
        sendEncCount(count);
        I2CFlag = false;
        break;
    }
  }

}
//update the current encoder count
int getEncCount(){
  return 0;
}

void forward(int speed){
  digitalWrite(NSLEEP, HIGH); //  nSleep should be kept high for normal operation
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, speed);
}

void reverse(int speed){
  digitalWrite(NSLEEP, HIGH);
  analogWrite(INPUT1, speed);
  analogWrite(INPUT2, 0);
}

void brake(){
  digitalWrite(NSLEEP, HIGH);
  analogWrite(INPUT1, 255);
  analogWrite(INPUT2, 255);
}

/**
 * Callback function upon receiving a request for data via I2C from master
 * This will request a set number of bytes as a message that will be formed when its time 
 */
void requestEvent() {
  I2CFlag==true;
  //send message - i guess this will be global variables we are reguarly updating or something hmmmm 
  samiData = 0;
  Wire.write(samiData);
}

//callback function for recieving messages and setting the appropriate status
void msgEvent(int numBytes){
 I2CFlag = true;
 while (1 < Wire.available()) { // loop through all but the last
    int x = Wire.read(); // receive byte as a character
    Serial.print(x); // print the character
    I2Cstatus = x;
  }

}

//will be used for sending the count - may be a more generic sending function later idk 
void sendEncCount(int count){
  
}
