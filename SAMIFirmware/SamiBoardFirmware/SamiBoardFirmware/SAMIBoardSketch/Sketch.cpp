#include <Arduino.h>
#include <Wire.h>
#include <PID_v2.h>

//function headers
void reverse();
void forward();
void encoderISR();
void receiveMotorCommand();

//I2C
const int I2C_ID = 1;

// MOTOR INPUTS
const int INPUT1_PIN = 13; //HIN1
const int INPUT2_PIN = 14; //LIN1
const int INPUT3_PIN = 9;  //HIN2
const int INPUT4_PIN = 10; //LIN2

//CURRENT SENSOR
const int CURRENT_IN_PIN = 19; //output from current sensor

//I2C
const int SDA_PIN = 27;
const int SCL_PIN = 28;

//HALL EFFECT ENCODERS
const int Q1_PIN = 32;
const int Q2_PIN = 1;
const char X = 5;
long encoderCount = 0;
char encoderArray[4][4] = { //quadrature decoding map  ->  0 = unchanged, 1 = increment, -1 = decrement, X = invalid
//  00  01  10  11
	{0, -1, 1, X},  // 00 
	{1, 0, X, -1},  // 01 
	{-1, X, 0, 1},  // 10
	{X, 1, -1, 0}   // 11
}; 
int errorCount = 0;
int oldValue = 0;

//MOTOR PID
float motorOutput = 0;
float motorError = 0;
float motorErrorSum = 0;
float motorPreviousError = 0;
float motorDesiredPos = 0;
double Kp = 0;
double Ki = 0;
double Kd = 0;

PID m1pid(&encoderCount, &motorOutput, &motorDesiredPos, Kp, Ki, Kd, DIRECT); //PID object


void setup() {
	
  Serial.begin(115200);
  Wire.begin(I2C_ID);
  Wire.onReceive(receiveMotorCommand); //create event when command is given to motor driver board
  
  pinMode(INPUT1_PIN, OUTPUT);
  pinMode(INPUT2_PIN, OUTPUT);
  pinMode(INPUT3_PIN, OUTPUT);
  pinMode(INPUT4_PIN, OUTPUT);
  pinMode(CURRENT_IN_PIN, INPUT);
  // hall effect sensor ISRs are the same so we can make use of increased resolution and activate changes more often
  attachInterrupt(Q1_PIN, encoderISR, CHANGE);
  attachInterrupt(Q2_PIN, encoderISR, CHANGE);
}

void loop() {
	int startTime = millis();
	while(startTime + 5000 > millis()) {
		forward();
	}
	startTime = millis();
	while(startTime + 5000 > millis()) {
		brake();
	}
	startTime = millis();
	while(startTime + 5000 > millis()) {
		reverse();
	}
	startTime = millis();
	while(startTime + 5000 > millis()) {
		brake();
	}
	startTime = millis();
	
	//Serial.print(readCurrentSensor());
}

/************************************************************************/
/* Read the current sensor and act upon reading                         */
/************************************************************************/
int readCurrentSensor() {
	int currentVal = analogRead(CURRENT_IN_PIN);
	return currentVal;
}

/************************************************************************/
/* Interrupt Service Routine for encoder value changes                  */
/************************************************************************/
void encoderISR() {
	int readValue = (digitalRead(Q2_PIN) << 1) | digitalRead(Q1_PIN); //bitshift values for array indexing
	char mappedValue = encoderArray[oldValue][readValue];
	if(mappedValue == X) {
		errorCount++;
	}
	else {
		encoderCount -= value; //positive value = decreasing encoder count? 
	}
	oldValue = readValue;
	
}

/************************************************************************/
/* Function to move motors forward using A3910 Half Bridge Truth Table                                                                     */
/************************************************************************/
void forward(){
	digitalWrite(INPUT1_PIN, HIGH);
	digitalWrite(INPUT2_PIN, LOW);
	digitalWrite(INPUT3_PIN, LOW);
	digitalWrite(INPUT4_PIN, LOW);
}

/************************************************************************/
/* Function to move motors backward (manually swap polarity) using A3910 Half Bridge Truth Table                                                                     */
/************************************************************************/
void reverse(){
	digitalWrite(INPUT1_PIN, LOW);
	digitalWrite(INPUT2_PIN, LOW);
	digitalWrite(INPUT3_PIN, HIGH);
	digitalWrite(INPUT4_PIN, LOW);
}

/************************************************************************/
/* Function to brake motors using A3910 Half Bridge Truth Table                                                                     */
/************************************************************************/
void brake(){
	digitalWrite(INPUT1_PIN, LOW);
	digitalWrite(INPUT2_PIN, HIGH);
	digitalWrite(INPUT3_PIN, LOW);
	digitalWrite(INPUT4_PIN, HIGH);
}


void receiveMotorCommand() {
	
}