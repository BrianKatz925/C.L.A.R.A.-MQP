/**
   To be uploaded on motor driver module boards
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
#include <Encoder.h>

#define NSLEEP 5
#define INPUT1 9
#define INPUT2 10
#define currentRead 17 //26
#define enc1 2 //32
#define enc2 3 //1 

char I2Cstatus = '0';
int slow = 100; //default slow speed
int fast = 500; //default fast speed
int count = 0; //for encoder count

//Encoder Setup
Encoder motor(enc1, enc2);
long motorPosition = 0;
int motorSpeed = 0;

void setup() {
  //Serial.begin(9600); //begin Serial - lower baud rates work better for AtMega328 board

  //set up I2C address as 0x01 for the current board - in the future this will be sequential for all boards so the master can address them individually
  Wire.begin(0x02);
  pinMode(currentRead, INPUT);
  pinMode(enc1, INPUT);
  pinMode(enc2, INPUT);

  //upon receiving a request from the master, call requestEvent
  Wire.onRequest(requestEvent);
  Wire.onReceive(msgEvent);
  digitalWrite(NSLEEP, HIGH); //  nSleep should be kept high for normal operation
}

int encInterval = 5;
int lastTime = 0;

void loop() {
  if (millis() - lastTime >= encInterval) {
    getEncCount();
    readCurrent();
    lastTime = millis();
  }
}


int motorCurrent = 0;

void readCurrent() {
  motorCurrent = analogRead(currentRead) * 255 / 1023;
}

long difference, newRead;
int rpm;

void getEncCount() {
  newRead = motor.read(); //read the encoder
  difference = newRead - motorPosition; //get the difference
  rpm = difference * 12 * 60 / 150; //RPM
  motorPosition = newRead;//update last position
  motorSpeed = rpm; // newRead; //set the speed
}

void forward(int speed) {
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, speed);
}

void reverse(int speed) {
  analogWrite(INPUT1, speed);
  analogWrite(INPUT2, 0);
}

void brake() {
  analogWrite(INPUT1, 255);
  analogWrite(INPUT2, 255);
}

/**
   Callback function upon receiving a request for data via I2C from master
   This will request a set number of bytes as a message that will be formed when its time
*/

char data[2]; // size of how many pieces of data we want to send
void requestEvent() {
  //write once with an array of multiple bytes
  data[0] = motorSpeed;
  data[1] = motorCurrent;
  //Wire.write(data);
  Wire.write(motorSpeed);
  Wire.write(motorCurrent);
}

//callback function for recieving messages and setting the appropriate status
void msgEvent(int numBytes) {
  // I2CFlag = true;
  while (Wire.available() > 0) { // loop through all but the last
    int x = Wire.read(); // receive byte as a character
    //    if (x>127){
    //      x = 256-x;
    //      x*=-1;
    //    }
    I2Cstatus = x;
  }
  switch (I2Cstatus) {
    default: //no message- status defaults to zero
      break;
    case 0: //motor stop
      brake();

      break;
    case 2: //motor forward slowly
      
      break;
    case 9:
      reverse(255);
      break;
    case 8:
      forward(255);
      break;

  }

}
