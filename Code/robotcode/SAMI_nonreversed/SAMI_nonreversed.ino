/**
   To be uploaded on motor driver module boards - Handle motor motion and control internally
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


/**********************************************************************************************************************

                                                    VARIABLES + DEFINITIONS


 **********************************************************************************************************************/

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
//Motor speed definitions
int slow = 100; //default slow speed
int fast = 500; //default fast speed
int fastSpeed = 255; //cabl75e motor speeds - this gets changed throughout the program
int slowRPM = 75;
int fastRPM = 25;

//Current Sensor variables
float stall_current = 0.00; // 0.25; //current value when motor is stalled
float motorCurrent = 0.00; //data variable representing motor current sent through I2C to mainboard
//bool motorstalled;
long lastTime = 0; //previous time current was checked
bool motorstalleddown, motorstalledup; //motor stalled variables

//Encoder variables
volatile int16_t count = 0; //current encoder count - sent through I2C to mainboard

//I2C Variables
byte address = 0x07; //the address of the board being flashed
char I2Cstatus = '0'; //I2C command sent from Mainboard
byte data[4]; //the data variable to be sent along I2C

//PID variables
float kp = 0.2; //proportional constant
int result = 0; //PID result placeholder variable
int lastcount = 0; //placeholder for previous count of encoder ticks
int motSpeed = 0; //motor speed, RPM
int countdiff = 0; //difference in encoder count from lastcount to current count
int calcSpeedPID = 0; //PID calculation result
const int Ngear = 298; //motor gear ratio
int currenterror = 0; //current motor speed error for PID
int targetSpeed = 0; //target speed in RPM
int cablelength = 0; //cable length in m



/**********************************************************************************************************************

                                                         SETUP + LOOP


 **********************************************************************************************************************/

void setup() {
  //set up I2C for the given address
  Wire.begin(address);

  // set up sensor pins
  pinMode(currentRead, INPUT);
  pinMode(enc1, INPUT);
  pinMode(enc2, INPUT);

  //attach interrupts on both encoders w/ isr callback functions for each of them - speeds up ISRs
  attachInterrupt(digitalPinToInterrupt(enc1), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2), isr2, CHANGE);

  //set up I2C event channels
  Wire.onRequest(requestEvent); //upon receiving a request from the master, call requestEvent
  Wire.onReceive(msgEvent); //upon receiving a message the I2C device, call msgEvent

  digitalWrite(NSLEEP, HIGH); //  nSleep should be kept high for normal operation of the MD9927 motor driver

  //if the current address is one of the cable motors (4,5,6) - reduce maximum speed to 150 for more control
  if (address == 0x04 | address == 0x05 | address == 0x06 ) {
    fastSpeed = 150;
  }
}


void loop() {
  if ((millis() - lastTime) >= 10) { //calculate PID every 20ms

    //calculate current RPM and compute PID with it
    countdiff = count - lastcount; //difference in encoder count
    lastcount = count;

    //calculate motor speed in RPM
    motSpeed = abs(countdiff * 1.67); //((1000*60)/(12*20*Ngear))) i dont know why it hates actual math.u.. ;
    currenterror = targetSpeed - motSpeed; //calculate current error from our target speed
    calcSpeedPID = calcPID(currenterror); //calculate PID
    //cablelength = calcCablelen(count)*10;

    lastTime = millis(); //update timer
  }

  //check the current sensor
  readCurrent();

  //switch statement given i2c input command from command line or controller
  switch (I2Cstatus) {
    default: //no message- status defaults to zero
      break;
    case 0: //motor stop
      brake();
      break;
    case 1: //lead screw up speed
      reverse(250);
      break;
    case 2: //lead screw down speed
      forward(250);
      break;
    case 3: //if motor is stalled, brake
      if (motorstalleddown) {
        brake();
        //delay(200);
        I2Cstatus = 0;

      } else {
        forward(150);
      }
      break;
    case 4: //if motor is stalled, brake
      if (motorstalledup) {
        brake();
        //delay(200);
        I2Cstatus = 0;

      } else {
        reverse(150);
      }
      break;
    case 9: //cable motor
      reverse(200);
      break;
    case 8: //cable motor
      forward(200);
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
    case 14: //attempt piD position
      long pidtarget = 10000;
      if (pidposition(pidtarget)) {

        brake;
        I2Cstatus = 0;
      }
      break;
  }
}
bool pidposition(long pidtarget) {
  float ppos = 0.1;
  float ipos = 0.0;
  float dpos = 0.0;

  long error = pidtarget - count;
  uint16_t adjeffort = ppos * error; //just p for now, I and D are for losers (and better tuned systems)
  adjeffort = abs(max(min(100, adjeffort), 0));
  if (error >= 0) {
    reverse(adjeffort);
  }
  else {
    forward(adjeffort);
  }
  if (abs(error) < 100) {
    return true;
  } else {
    return false;
  }

}

/**********************************************************************************************************************

                                                        INTERRUPT SERVICE ROUTINES


 **********************************************************************************************************************/

//ISRs split up for each hall-effect latch sensor to decrease processing time within the ISR and allow I2C to work every time without interruption

/**
   ISR for encoder 1 - reads registers instead of digital read for increased speed and increments/decrements a count variable
*/
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

/**
   ISR for encoder 2 - reads registers instead of digital read for increased speed and increments/decrements a count variable
*/
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

/**********************************************************************************************************************

                                                    CURRENT SENSOR FUNCTIONS

 **********************************************************************************************************************/

/**
   Function to set the motorCurrent variable to the current sensor reading and check if it is stalled
*/
void readCurrent() {
  motorCurrent = analogRead(currentRead);
  // motorstalled=true;
  if (motorCurrent > 400) {
    motorstalledup = true;
  }
  else if (motorCurrent > 390) { //if the current reading is above the preset stall current value, brake the motor
    motorstalleddown = true;
  } else {
    motorstalleddown = false;
    motorstalledup = false;
  }
}

/**
   Function to home the cables using the current threshold
   Once homed, the motors reverse to limit current
   @param currentthreshold - the float current threshold in A
*/
void homeCables(float currentthreshold) {
  if (motorCurrent >= currentthreshold) {
    reverse(150);
  }
}

/**********************************************************************************************************************

                                                   DRIVING FUNCTIONS

 **********************************************************************************************************************/

/**
   This function drives the motors forward
   @param speed - the integer speed of the motor
*/
void forward(int speed) {
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, speed);
}

/**
   This function drives the motors reverse
   @param speed - the integer speed of the motor
*/
void reverse(int speed) {
  analogWrite(INPUT1, speed);
  analogWrite(INPUT2, 0);
}

/**
   This function stops the motors
*/
void brake() {
  analogWrite(INPUT1, 255);
  analogWrite(INPUT2, 255);
}



/**
   This function calculates PID for the motor speed
   @param error - the given error
   @return result - an integer clamped value for the speed of the motor
*/
int calcPID(int error) {
  result = abs(error * kp);
  if (error < 0) {
    result -= result;
  }
  else {
    result += result;
  }
  result = max(min((result), 200), 0);
  return result;
}

/**********************************************************************************************************************

                                                    I2C Functions


 **********************************************************************************************************************/

/**
   Callback function upon receiving a request for data via I2C from master
   This will request a set number of bytes as a message that will be formed for interpretation by the mainboard
*/
void requestEvent() {
  //split the int encoder count into multiple bytes
  data[0] = (count >> 8) & 0xFF;
  data[1] = count & 0xFF;
  data[2] = motorCurrent;
  data[3] = motSpeed;
  Wire.write(data, 4);
}

/**
   callback function for recieving messages and setting the appropriate I2C status
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
