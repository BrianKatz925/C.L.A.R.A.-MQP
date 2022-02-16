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
#include <PIDController.h>

/*******************************************
     PIN DEFINITIONS - Arduino Nano pins
 *****************************************/
#define NSLEEP 5
#define INPUT1 9
#define INPUT2 10
#define currentRead 17 //26
#define enc1 2 //32
#define enc2 3 //1 

PIDController pidSpeed;
/******************************************
                 VARIABLES
 *****************************************/
char I2Cstatus = '0'; //I2C command sent from Mainboard

//Motor speed definitions
int slow = 100; //default slow speed
int fast = 500; //default fast speed

//Current Sensor variables
const float stall_current = 35; //current value when motor is stalled
int motorCurrent = 0; //data variable representing motor current sent through I2C to mainboard
bool motorstalled = false;
int lastTime = 0; //previous time current was checked

//Quadrature encoder constants
const int X = 5; //invalid state definition
int encoderArray[4][4] = { //state matrix
  {0, -1, 1, X},
  {1, 0, X, -1},
  { -1, X, 0, 1},
  {X, 1, -1, 0}
};

int newValue; //integer between 0 and 3 representing quadrature encoder state
int error = 0; //count of 'X's or invalid encoder states
int oldValue = 0; //previous encoder reading
int count = 0; //current encoder count - sent through I2C to mainboard
byte address = 0x06;
int fastSpeed = 255;

void setup() {
  //set up I2C address as 0x01 for the current board - in the future this will be sequential for all boards so the master can address them individually
  Wire.begin(address);

  // set up sensor pins
  pinMode(currentRead, INPUT);
  pinMode(enc1, INPUT);
  pinMode(enc2, INPUT);

  //attach interrupts on both encoders w/ isr callback function
  //attachInterrupt(digitalPinToInterrupt(enc1), isr, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(enc2), isr, CHANGE);

  //set up I2C event channels
  Wire.onRequest(requestEvent); //upon receiving a request from the master, call requestEvent
  Wire.onReceive(msgEvent); //upon receiving a message the I2C device, call msgEvent

  digitalWrite(NSLEEP, HIGH); //  nSleep should be kept high for normal operation of the MD9927 motor driver

  pidSpeed.begin();
  pidSpeed.tune(1, 0, 0);
  pidSpeed.limit(0, 255);

  if (address == 0x04 | address == 0x05 | address == 0x06 ){
    fastSpeed = 150;
  }
}

int lastcount = 0;
int motSpeed;
int countdiff = 0;
int calcSpeedPID;
int Ngear = 150;
void loop() {

  if ((millis() - lastTime) >= 20) { //if 20 ms passed since the last reading, read the current
    readCurrent();
    //calculate current RPM and compute PID with it
    countdiff = count - lastcount;
    motSpeed = (countdiff / 12) * (1000 / 1) * (60 / 1) / Ngear;
    calcSpeedPID = pidSpeed.compute(motSpeed);
    lastTime = millis();
  }

  if (motorCurrent >= stall_current) { //if the current reading is above the preset stall current value, break the motor
    motorstalled == true;
    brake();
    I2Cstatus = 0;
  }
  //increment encoders without ISR
  newValue = (digitalRead(enc1) << 1) | digitalRead(enc2); 
  switch (oldValue)
  {
    case 0:
      switch (newValue)
      {
        case 0: break;
        case 1: count--; break;
        case 2: count++; break;
        case 3: error++; break;
      }
      break;
    case 1:
      switch (newValue)
      {
        case 0: count++; break;
        case 1: break;
        case 2: error++; break;
        case 3: count--; break;
      }
      break;
    case 2:
      switch (newValue)
      {
        case 0: count--; break;
        case 1: error++; break;
        case 2: break;
        case 3: count++; break;
      }
      break;
    case 3:
      switch (newValue)
      {
        case 0: error++; break;
        case 1: count++; break;
        case 2: count--; break;
        case 3: break;
      }
      break;
  }
  oldValue = newValue;

  int slowRPM = 25;
  int fastRPM = 50;
  //switch statement given i2c input command from command line or controller
  switch (I2Cstatus) {
    default: //no message- status defaults to zero
      break;
    case 0: //motor stop
      brake();
      break;
    case 1: //lead screw up speed
      pidSpeed.setpoint(slowRPM); //check all the signs
      forward(100);
      break;
    case 2: //lead screw down speed
      pidSpeed.setpoint(slowRPM);
      reverse(calcSpeedPID);
      break;
    case 9:
      pidSpeed.setpoint(-slowRPM);
      reverse(255);
      break;
    case 8:
      pidSpeed.setpoint(fastRPM);
      forward(255);
      break;
    case 12: //cable down speed
      pidSpeed.setpoint(fastSpeed);
      forward(fastSpeed);
      break;
    case 13: //cable down speed
      pidSpeed.setpoint(fastSpeed);
      reverse(fastSpeed);
      break;
  }

}

/***********************************
         HELPER FUNCTIONS
 *******************************/

/**
   Function to set the motorCurrent variable to the current sensor reading
*/
void readCurrent() {
  motorCurrent = analogRead(currentRead) * 255 / 1023; //multiply by AD
}

void homeCables(float currentthreshold){
  if (motorCurrent >= currentthreshold){
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
byte data[3];
void requestEvent() {
  //split the int encoder count into multiple bytes
  data[0] = (count >> 8) & 0xFF;
  data[1] = count & 0xFF;
  data[2] = motorCurrent;
  //Write encoder count and current values along i2C for a request
  Wire.write(data, 3);
}

/**
   callback function for recieving messages and setting the appropriate status
*/
void msgEvent(int numBytes) {
  noInterrupts();
  // I2CFlag = true;
  while (Wire.available() > 0) { // loop through all but the last
    int x = Wire.read(); // receive byte as a character
    //    if (x>127){
    //      x = 256-x;
    //      x*=-1;
    //    }
    I2Cstatus = x;
  }
  interrupts();
}
