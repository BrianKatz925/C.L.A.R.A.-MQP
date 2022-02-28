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

/*******************************************
     PIN DEFINITIONS - Arduino Nano pins
 *****************************************/
#define NSLEEP 5
#define INPUT1 9
#define INPUT2 10
#define currentRead 17 //26
#define enc1 2 //32
#define enc2 3 //1 

/******************************************
                 VARIABLES
 *****************************************/
char I2Cstatus = '0'; //I2C command sent from Mainboard

//Motor speed definitions
int slow = 100; //default slow speed
int fast = 500; //default fast speed

//Current Sensor variables
float stall_current = 0.00; // 0.25; //current value when motor is stalled
float motorCurrent = 0.00; //data variable representing motor current sent through I2C to mainboard
bool motorstalled;
long lastTime = 0; //previous time current was checked
bool motorstalleddown, motorstalledup;


volatile int16_t count = 0; //current encoder count - sent through I2C to mainboard
byte address = 0x05;
int fastSpeed = 255;

void setup() {
  //set up I2C address as 0x01 for the current board - in the future this will be sequential for all boards so the master can address them individually
  Wire.begin(address);

  // set up sensor pins
  pinMode(currentRead, INPUT);
  pinMode(enc1, INPUT);
  pinMode(enc2, INPUT);

  //attach interrupts on both encoders w/ isr callback function
  attachInterrupt(digitalPinToInterrupt(enc1), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2), isr2, CHANGE);

  //set up I2C event channels
  Wire.onRequest(requestEvent); //upon receiving a request from the master, call requestEvent
  Wire.onReceive(msgEvent); //upon receiving a message the I2C device, call msgEvent

  digitalWrite(NSLEEP, HIGH); //  nSleep should be kept high for normal operation of the MD9927 motor driver


  if (address == 0x04 | address == 0x05 | address == 0x06 ) {
    fastSpeed = 150;
  }
}

//i'm sorry lewin this is a P controller and i know it's disgusting
int kp = 4;
int result = 0;
int calcPID(int error) {
  result = abs(error * kp);
  if (error < 0) {
    result -= result;
  }
  else {
    result += result;
  }
  result = max(min((result), 255), 0);
  return result;
}

int lastcount = 0;
int motSpeed = 0;
int countdiff = 0;
int calcSpeedPID = 0;
int Ngear = 298;
int currenterror = 0;
int targetSpeed = 0;
int cablelength = 0;
void loop() {
  if ((millis() - lastTime) >= 20) { //if 20 ms passed since the last reading, read the current
    //calculate current RPM and compute PID with it
    countdiff = count - lastcount;
    lastcount = count;
    motSpeed = abs(countdiff * 0.84); //((1000*60)/(12*20*Ngear))) i dont know why it hates actual math.u.. ;
    currenterror = targetSpeed - motSpeed;
    calcSpeedPID = calcPID(currenterror);
    //cablelength = calcCablelen(count)*10;

    lastTime = millis();

  }

  //check the currentsensor
  readCurrent();

  int slowRPM = 35;
  int fastRPM = 25;
  //switch statement given i2c input command from command line or controller
  switch (I2Cstatus) {
    default: //no message- status defaults to zero
      break;
    case 0: //motor stop
      brake();
      break;
    case 1: //lead screw up speed
      reverse(150);
      break;
    case 2: //lead screw down speed
      forward(150);
      break;
    case 3:
      if (motorstalleddown) {
        brake();
        //delay(200);
        I2Cstatus = 0;

      } else {
        forward(130);
      }
      break;
    case 4:
      if (motorstalledup) {
        brake();
        //delay(200);
        I2Cstatus = 0;

      } else {
        reverse(130);
      }
      break;
    case 9:
      if (address == 0x01) {
        reverse(255);
      } else {
        reverse(255);
      }
      break;
    case 8:
      if (address == 0x01) {
        forward(255);
      } else {
        forward(255);
      }
      break;
    case 11:
      targetSpeed = slowRPM;
      if (address == 0x01) {
        reverse(calcSpeedPID);
      } else {
        reverse(calcSpeedPID);
      }
      break;
    case 10:
      targetSpeed = slowRPM;
      if (address == 0x01) {
        forward(calcSpeedPID);
      } else {
        forward(calcSpeedPID);
      }
      break;

    case 12: //cable up speed
      targetSpeed = slowRPM;
      forward(calcSpeedPID);
      break;
    case 13: //cable down speed
      targetSpeed = slowRPM;
      reverse(calcSpeedPID);
      break;
  }

}


//ISR for encoders
void isr1() {
  uint8_t encread1 = PIND >> 2 & 0x01;
  uint8_t encread2 = PIND >> 3 & 0x01;  //get the two bit encoder read
  if (encread1 == encread2) {
    count++;
  }
  else {
    count--;
  }
}
void isr2() {
  uint8_t encread1 = PIND >> 2 & 0x01;
  uint8_t encread2 = PIND >> 3 & 0x01;//get the two bit encoder read
  if (encread1 != encread2) {
    count++;
  }
  else {
    count--;
  }
}

/***********************************
         HELPER FUNCTIONS
 *******************************/

/**
   Function to set the motorCurrent variable to the current sensor reading
*/
void readCurrent() {
  motorCurrent = analogRead(currentRead); //multiply by AD
  // motorstalled=true;
  if (motorCurrent > 430) {
    motorstalledup = true;
  }
  else if (motorCurrent > 375) { //if the current reading is above the preset stall current value, break the motor
    motorstalleddown = true;
  } else {
    motorstalleddown = false;
    motorstalledup = false;
  }
}

void homeCables(float currentthreshold) {
  if (motorCurrent >= currentthreshold) {
    reverse(150);
  }
}

/***********************
   DRIVING FUNCTIONS
 *********************/

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
byte data[4];
void requestEvent() {
  //split the int encoder count into multiple bytes
  data[0] = (count >> 8) & 0xFF;
  data[1] = count & 0xFF;
  data[2] = motorCurrent;
  data[3] = motSpeed;
    Wire.write(data, 4);
  
}

/**
   callback function for recieving messages and setting the appropriate status
*/
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
}
