// Copyright 2020 Shoushan Chiang

#include <avr/io.h>

/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>
//#include <CapacitiveSensor.h>
// Beginning of Auto generated function prototypes by Atmel Studio
void initEncoders();
long readEncoder(int encoder);
void clearEncoderCount();
// End of Auto generated function prototypes by Atmel Studio

// CapacitiveSensor   cs_3_2 = CapacitiveSensor(3,2);        // 10M resistor
// between pins 3 & 2, pin 2 is sensor pin, add a wire and or foil if desired

const int MOTOR_ONE_DIR_PIN = 4;
const int MOTOR_ONE_PWM_PIN = 5;
const int MOTOR_TWO_DIR_PIN = 6;
const int MOTOR_TWO_PWM_PIN = 9;
const int MOTOR_THREE_DIR_PIN = 12;
const int MOTOR_THREE_PWM_PIN = 10;
// const int slaveSelectEnc1 = 13;
const int slaveSelectEnc2 = 8;
const int slaveSelectEnc3 = 7;
const int MOTOR_BRAKE_ENABLE_PIN = 11;

const int MOTOR_FOUR_PWM_PIN = 13;
// const int MOTOR_ONE_CUR_SENSE_PIN = 1;
// const int MOTOR_TWO_CUR_SENSE_PIN = 2;
// const int MOTOR_THREE_CUR_SENSE_PIN = 3;
// int cur_sense1;
// int prev_cur_sense1;
// int contact_current_thres = 400; // this will depend on the gain of the
// op-amp used for amplifying the current sensing int touch_sense_thres = 1000;
// int contact_count = 0;

float gear_ratio = 300.0;  // for 1000:1 gear ratio, 986*12 = 11832 = 1 rotation
int encoder_ticks_per_rev = 14;
float encoder_ticks_to_angle =
    (360 / (float)(gear_ratio * encoder_ticks_per_rev));  // in degrees

double delta_t = 0.01;  // in seconds
double ticks = 0;
double ticks_counted = 0;
double ticks_two = 0;
double ticks_counted_two = 0;
double ticks_three = 0;
double ticks_counted_three = 0;

double ticks_four = 0;
double ticks_counted_four = 0;

float motor_one_output = 0.0;
double mo = 0.0;
double mo2 = 0.0;
double mo3 = 0.0;

double mo4 = 0.0;

byte MOTOR_ONE_CUR_DIR_COMMAND = 1;
byte MOTOR_TWO_CUR_DIR_COMMAND = 1;
byte MOTOR_THREE_CUR_DIR_COMMAND = 1;

byte MOTOR_FOUR_CUR_DIR_COMMAND = 1;

byte MOTOR_ONE_PREV_DIR_COMMAND = MOTOR_ONE_CUR_DIR_COMMAND;
byte MOTOR_TWO_PREV_DIR_COMMAND = MOTOR_TWO_CUR_DIR_COMMAND;
byte MOTOR_THREE_PREV_DIR_COMMAND = MOTOR_THREE_CUR_DIR_COMMAND;

byte MOTOR_FOUR_PREV_DIR_COMMAND = MOTOR_FOUR_CUR_DIR_COMMAND;

int i = 0;

double motor_one_error;
double motor_one_error_sum = 0;
double motor_one_prev_error;  // lastErrorL;

double motor_prev_pos = 0;
double desired_angle = 0;
volatile double motor_cur_pos = 0;
double t = 0.0;
float pi = 3.14159;
volatile double motor_one_des_length = 0;
volatile double motor_two_des_length = 0;
volatile double motor_three_des_length = 0;

volatile double motor_four_des_length = 0;

double des_length1 = 0;
double des_length2 = 0;
double des_length3 = 0;

double des_length4 = 0;

// byte motorCommand[3];
// byte encoderData[3];

byte motorCommand[4];
byte encoderData[4];

byte _ready = 1;
byte notready = 0;

/// NEWEST PID PARAMETERS
double Kp1 = 0.5, Ki1 = 0.1, Kd1 = 0.0005;
double Kp2 = 0.5, Ki2 = 0.1, Kd2 = 0.0005;
double Kp3 = 0.5, Ki3 = 0.1, Kd3 = 0.0005;

double Kp4 = 0.5, Ki4 = 0.1, Kd4 = 0.0005;

unsigned long loop_index = 0;
signed long encoder1count = 0;
signed long encoder2count = 0;
signed long encoder3count = 0;

signed long encoder4count = 0;

double encodercount = 0;
double encodercount2 = 0;
double encodercount3 = 0;

double encodercount4 = 0;

bool timer5_flag = false;
bool switch_direction_buffer_state = false;
long timer5_count = 0;

// PID m1pid(&ticks, &mo, &des_length1, Kp1, Ki1, Kd1, DIRECT);
PID m1pid(&encodercount, &mo, &des_length1, Kp1, Ki1, Kd1, DIRECT);
PID m2pid(&encodercount2, &mo2, &des_length2, Kp2, Ki2, Kd2, DIRECT);
PID m3pid(&encodercount3, &mo3, &des_length3, Kp3, Ki3, Kd3, DIRECT);

PID m4pid(&encodercount4, &mo4, &des_length4, Kp4, Ki4, Kd4, DIRECT);

int motor1_cur_val = 0;
int motor2_cur_val = 0;
int motor3_cur_val = 0;

int motor4_cur_val = 0;

double m1_cur_val = 0.0;
double m2_cur_val = 0.0;
double m3_cur_val = 0.0;

double m4_cur_val = 0.0;

double m1_prev_val = 0.0;
double m2_prev_val = 0.0;
double m3_prev_val = 0.0;

double m4_prev_val = 0.0;

double alpha = 0.1;

int count = 0;
byte motor_state = 0;

static inline void initTimers(void) {
    // Init timers for motor PWM using Fast PWM mode, use this instead of analog
    // to get changeable PWM frequency

    // Timer 1 A,B
    TCCR1A |= (1 << WGM10);  /* Fast PWM mode, 8-bit */
    TCCR1B |= (1 << WGM12);  /* Fast PWM mode, pt.2 */
    TCCR1B |= (1 << CS10);   /* PWM Freq = F_CPU//256, F_CPU = 16,000,000 in our
                                case, this setup gives 7.8 kHz*/
    TCCR1A |= (1 << COM1A1); /* PWM output on OCR1A */
    TCCR1A |= (1 << COM1B1); /* PWM output on OCR1B */

    // Timer 3 A
    TCCR3A |= (1 << WGM10);  /* Fast PWM mode, 8-bit */
    TCCR3B |= (1 << WGM12);  /* Fast PWM mode, pt.2 */
    TCCR3B |= (1 << CS30);   /* PWM Freq = F_CPU/1/256 */
    TCCR3A |= (1 << COM3A1); /* PWM output on OCR3A */

    // Timer 4 A
    TCCR4A |= (1 << PWM4A);  /* Fast PWM mode, 8-bit */
    TCCR4D |= (0 << WGM40);  /* Fast PWM mode, pt.2 */
    TCCR4B |= (1 << CS40);   /* PWM Freq = F_CPU/1/256 */
    TCCR4A |= (1 << COM4A1); /* PWM output on OCR4a */
    OCR4C = 255;
}

void setup() {
    //  Wire.begin(8);
    //  Wire.onReceive(receiveMotorCommand); // register event
    //  Wire.onRequest(requestEncoderData);
    Serial1.begin(9600);
    pinMode(MOTOR_ONE_DIR_PIN, OUTPUT);
    pinMode(MOTOR_ONE_PWM_PIN, OUTPUT);
    pinMode(MOTOR_TWO_DIR_PIN, OUTPUT);
    pinMode(MOTOR_TWO_PWM_PIN, OUTPUT);
    pinMode(MOTOR_THREE_DIR_PIN, OUTPUT);
    pinMode(MOTOR_THREE_PWM_PIN, OUTPUT);

    pinMode(MOTOR_FOUR_PWM_PIN, OUTPUT);
    //  setPwmFrequency(5, 8);
    m1pid.SetMode(AUTOMATIC);
    m1pid.SetSampleTime(2);
    m1pid.SetOutputLimits(-255, 255);
    m2pid.SetMode(AUTOMATIC);
    m2pid.SetSampleTime(2);
    m2pid.SetOutputLimits(-255, 255);
    m3pid.SetMode(AUTOMATIC);
    m3pid.SetSampleTime(2);
    m3pid.SetOutputLimits(-255, 255);

    m4pid.SetMode(AUTOMATIC);
    m4pid.SetSampleTime(2);
    m4pid.SetOutputLimits(-255, 255);

    initEncoders();
    Serial1.println("Encoders Initialized...");
    delay(5);
    clearEncoderCount();
    Serial1.println("Encoders Cleared...");

    //  cs_3_2.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate
    //  on channel 1 - just as an example
    initTimers();
}

void loop() {
    //  int pot_val = analogRead(0);
    //  int full_range = 3.0*12*gear_ratio;
    //  des_length1 = map(pot_val, 0, 1023, 0, full_range);
    count = count + 1;
    if (count < 200) {
        des_length1 = 5.0 * 14 * gear_ratio;
        des_length2 = des_length1;
        des_length3 = des_length1;

        des_length4 = des_length1;

        motor_state = 0;
    } else if (count < 400) {
        des_length1 = 0.0 * 14 * gear_ratio;
        des_length2 = des_length1;
        des_length3 = des_length1;

        des_length4 = des_length1;

        motor_state = 1;
    } else {
        count = 0;
    }

    //  t = map(pot_val, 0, 1023, 0, 2*pi);
    //  t = t + pi/4;

    ///////////////////////////////////////////////////////
    // if (timer5_count < 500)
    //{
    //  if (timer5_count <= 50)
    //  {
    //    switch_direction_buffer_state = true;
    //  }
    //  else
    //  {
    //    switch_direction_buffer_state = false;
    //  }
    //  des_length1 = 2.0*12*gear_ratio;
    //  des_length2 = 0.0*12*gear_ratio;
    //  des_length3 = 0.0*12*gear_ratio;
    //  }
    //
    // if (timer5_count >= 500)
    //{
    //  if (timer5_count <= 550)
    //  {
    //    switch_direction_buffer_state = true;
    //  }
    //  else
    //  {
    //    switch_direction_buffer_state = false;
    //  }
    //  des_length1 = 0.0;
    //  des_length2 = 0.0;
    //  des_length3 = 0.0;
    //  }
    //
    // if (timer5_count >= 1000)
    //  {
    //    timer5_count = 0;
    //  }
    /////////////////////////////////////////////

    encoder1count = readEncoder(1);
    encodercount = (double)encoder1count;
    encoder2count = readEncoder(2);
    encodercount2 = (double)encoder2count;
    encoder3count = readEncoder(3);
    encodercount3 = (double)encoder3count;

    encoder4count = readEncoder(4);
    encodercount4 = (double)encoder4count;

    m1pid.Compute();
    m2pid.Compute();
    m3pid.Compute();

    m4pid.Compute();

    if (mo < 0) {
        MOTOR_ONE_CUR_DIR_COMMAND = 0;
        digitalWrite(MOTOR_ONE_DIR_PIN, LOW);
    } else {
        MOTOR_ONE_CUR_DIR_COMMAND = 1;
        digitalWrite(MOTOR_ONE_DIR_PIN, HIGH);
    }

    if (mo2 < 0) {
        MOTOR_TWO_CUR_DIR_COMMAND = 0;
        digitalWrite(MOTOR_TWO_DIR_PIN, LOW);
    } else {
        MOTOR_TWO_CUR_DIR_COMMAND = 1;
        digitalWrite(MOTOR_TWO_DIR_PIN, HIGH);
    }

    if (mo3 < 0) {
        MOTOR_THREE_CUR_DIR_COMMAND = 0;
        digitalWrite(MOTOR_THREE_DIR_PIN, LOW);
    } else {
        MOTOR_THREE_CUR_DIR_COMMAND = 1;
        digitalWrite(MOTOR_THREE_DIR_PIN, HIGH);
    }

    if (mo4 < 0) {
        MOTOR_FOUR_CUR_DIR_COMMAND = 0;
        TXLED1;
    } else {
        MOTOR_FOUR_CUR_DIR_COMMAND = 1;
        TXLED0;
    }

    // digitalWrite(slaveSelectEnc1, LOW);
    // digitalWrite(slaveSelectEnc2, LOW);
    // digitalWrite(slaveSelectEnc3, LOW);

    // digitalWrite(MOTOR_ONE_DIR_PIN, LOW);
    // analogWrite(MOTOR_ONE_PWM_PIN, 255);
    // digitalWrite(MOTOR_TWO_DIR_PIN, LOW);
    // analogWrite(MOTOR_TWO_PWM_PIN, 255);
    // digitalWrite(MOTOR_THREE_DIR_PIN, LOW);
    // analogWrite(MOTOR_THREE_PWM_PIN, 255);

    // analogWrite(MOTOR_ONE_PWM_PIN, abs(mo));
    // analogWrite(MOTOR_TWO_PWM_PIN, abs(mo2));
    // analogWrite(MOTOR_THREE_PWM_PIN, abs(mo3));

    OCR3A = abs(mo);   // Send PWM duty cycle to register associate with pin
                       // connected to motor 1
    OCR1A = abs(mo2);  // Send PWM duty cycle to register associate with pin
                       // connected to motor 2
    OCR1B = abs(mo3);  // Send PWM duty cycle to register associate with pin
                       // connected to motor 3

    OCR4A = abs(mo4);  // Send PWM duty cycle to register associate with pin
                       // connected to motor 4

    ////// MOTOR CURRENT READING
    // for motor1 (A0), motor2 (A1), motor3 (A2)
    motor1_cur_val = analogRead(A0);
    motor2_cur_val = analogRead(A1);
    motor3_cur_val = analogRead(A2);

    m1_cur_val = alpha * motor1_cur_val + (1 - alpha) * m1_cur_val;
    m2_cur_val = alpha * motor2_cur_val + (1 - alpha) * m2_cur_val;
    m3_cur_val = alpha * motor3_cur_val + (1 - alpha) * m3_cur_val;

    Serial1.print("try_me:  ");
    Serial1.println("ahahaha!!");
    /*Serial1.print(motor_state);
    //delay(5);
    Serial1.print("\t\t");
    Serial1.print(encodercount);
    Serial1.print("\t");
    Serial1.print(encodercount2);
    Serial1.print("\t");
    Serial1.print(encodercount3);
    Serial1.print("\t\t");
    Serial1.print(motor1_cur_val);
    Serial1.print("\t");
    Serial1.print(motor2_cur_val);
    Serial1.print("\t");
    Serial1.print(motor3_cur_val);
    Serial1.print("\t\t");
    Serial1.print(m1_cur_val);
    Serial1.print("\t");
    Serial1.print(m2_cur_val);
    Serial1.print("\t");
    Serial1.println(m3_cur_val);*/

    delay(5);
    //  long touch_sense_val =  cs_3_2.capacitiveSensor(30);

    // if (touch_sense_val >= touch_sense_thres)
    //{
    //  contact_count = contact_count + 1;
    ////  Serial.print("CONTACT DETECTED");
    ////  Serial.print("\t");
    ////  Serial.print("No of contact occurence detected:");
    ////  Serial.println(contact_count);
    //}
    //  Serial.println(touch_sense_val);

    // if (timer5_flag == true)
    //{
    ////  Serial.print(mo);
    ////  Serial.print("\t");
    ////  Serial.println(touch_sense_val);
    ////  Serial.println(encodercount);
    ////  Serial.println("");
    //  timer5_flag = false;
    //
    //////// This need to be commented otherwise the control won't work very
    ///well because the serial print slows down everything. 1 Serial print is
    ///okay if we want to check how good the PID control is. /
    ///Serial.print(des_length1); /  Serial.print("\t"); /
    ///Serial.print(des_length2); /  Serial.print("\t"); /
    ///Serial.print(des_length3); /  Serial.print("\t");
    //
    ////  Serial.print(encodercount);
    ////  Serial.print("\t");
    ////  Serial.print(encodercount2);
    ////  Serial.print("\t");
    ////  Serial.println(encodercount3);
    //
    ////Serial.print(mo);
    ////Serial.print("\t");
    ////Serial.print(mo2);
    ////Serial.print("\t");
    ////Serial.println(mo3);
    //
    //}
    //
    // prev_cur_sense1 = cur_sense1;
    // MOTOR_ONE_PREV_DIR_COMMAND = MOTOR_ONE_CUR_DIR_COMMAND;
    // MOTOR_TWO_PREV_DIR_COMMAND = MOTOR_TWO_CUR_DIR_COMMAND;
    // MOTOR_THREE_PREV_DIR_COMMAND = MOTOR_THREE_CUR_DIR_COMMAND;
    /*  Serial1.print(encodercount);
        Serial1.print("\t");
        Serial1.print(encodercount2);
        Serial1.print("\t");
        Serial1.println(encodercount3);
    */
}

void initEncoders() {
    // Set slave selects as outputs
    // pinMode(slaveSelectEnc1, OUTPUT);
    DDRE |= 0x04;  // pinMode PE2 OUTPUT
    pinMode(slaveSelectEnc2, OUTPUT);
    pinMode(slaveSelectEnc3, OUTPUT);

    pinMode(A3, OUTPUT);

    // Raise select pins
    // Communication begins when you drop the individual select signsl
    // digitalWrite(slaveSelectEnc1,HIGH);
    PORTE |= 0x04;  // Set PE2
    digitalWrite(slaveSelectEnc2, HIGH);
    digitalWrite(slaveSelectEnc3, HIGH);

    digitalWrite(A3, HIGH);

    SPI.begin();

    // Initialize encoder 1
    //    Clock division factor: 0
    //    Negative index input
    //    free-running count mode
    //    x4 quatrature count mode (four counts per quadrature cycle)
    // NOTE: For more information on commands, see datasheet
    // digitalWrite(slaveSelectEnc1,LOW);        // Begin SPI conversation
    PORTE &= 0xFB;       // Clear PE2
    SPI.transfer(0x88);  // Write to MDR0
    SPI.transfer(0x03);  // Configure to 4 byte mode
    // digitalWrite(slaveSelectEnc1,HIGH);       // Terminate SPI conversation
    PORTE |= 0x04;  // Set PE2

    // Initialize encoder 2
    //    Clock division factor: 0
    //    Negative index input
    //    free-running count mode
    //    x4 quatrature count mode (four counts per quadrature cycle)
    // NOTE: For more information on commands, see datasheet
    digitalWrite(slaveSelectEnc2, LOW);   // Begin SPI conversation
    SPI.transfer(0x88);                   // Write to MDR0
    SPI.transfer(0x03);                   // Configure to 4 byte mode
    digitalWrite(slaveSelectEnc2, HIGH);  // Terminate SPI conversation

    // Initialize encoder 3
    //    Clock division factor: 0
    //    Negative index input
    //    free-running count mode
    //    x4 quatrature count mode (four counts per quadrature cycle)
    // NOTE: For more information on commands, see datasheet
    digitalWrite(slaveSelectEnc3, LOW);   // Begin SPI conversation
    SPI.transfer(0x88);                   // Write to MDR0
    SPI.transfer(0x03);                   // Configure to 4 byte mode
    digitalWrite(slaveSelectEnc3, HIGH);  // Terminate SPI conversation

    // Initialize encoder 4
    //    Clock division factor: 0
    //    Negative index input
    //    free-running count mode
    //    x4 quatrature count mode (four counts per quadrature cycle)
    // NOTE: For more information on commands, see datasheet
    digitalWrite(A3, LOW);   // Begin SPI conversation
    SPI.transfer(0x88);      // Write to MDR0
    SPI.transfer(0x03);      // Configure to 4 byte mode
    digitalWrite(A3, HIGH);  // Terminate SPI conversation
}

long readEncoder(int encoder) {
    // Initialize temporary variables for SPI read
    unsigned int count_1, count_2, count_3, count_4;
    long count_value;

    // Read encoder 1
    if (encoder == 1) {
        // digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
        PORTE &= 0xFB;                 // Clear PE2
        SPI.transfer(0x60);            // Request count
        count_1 = SPI.transfer(0x00);  // Read highest order byte
        count_2 = SPI.transfer(0x00);
        count_3 = SPI.transfer(0x00);
        count_4 = SPI.transfer(0x00);  // Read lowest order byte
        // digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation
        PORTE |= 0x04;  // Set PE2
    }

    // Read encoder 2
    else if (encoder == 2) {
        digitalWrite(slaveSelectEnc2, LOW);  // Begin SPI conversation
        SPI.transfer(0x60);                  // Request count
        count_1 = SPI.transfer(0x00);        // Read highest order byte
        count_2 = SPI.transfer(0x00);
        count_3 = SPI.transfer(0x00);
        count_4 = SPI.transfer(0x00);         // Read lowest order byte
        digitalWrite(slaveSelectEnc2, HIGH);  // Terminate SPI conversation
    }

    else if (encoder == 3) {
        digitalWrite(slaveSelectEnc3, LOW);  // Begin SPI conversation
        SPI.transfer(0x60);                  // Request count
        count_1 = SPI.transfer(0x00);        // Read highest order byte
        count_2 = SPI.transfer(0x00);
        count_3 = SPI.transfer(0x00);
        count_4 = SPI.transfer(0x00);         // Read lowest order byte
        digitalWrite(slaveSelectEnc3, HIGH);  // Terminate SPI conversation
    }

    // Read encoder 4
    else if (encoder == 4) {
        digitalWrite(A3, LOW);         // Begin SPI conversation
        SPI.transfer(0x60);            // Request count
        count_1 = SPI.transfer(0x00);  // Read highest order byte
        count_2 = SPI.transfer(0x00);
        count_3 = SPI.transfer(0x00);
        count_4 = SPI.transfer(0x00);  // Read lowest order byte
        digitalWrite(A3, HIGH);        // Terminate SPI conversation
    }

    // Calculate encoder count
    count_value = (count_1 << 8) + count_2;
    count_value = (count_value << 8) + count_3;
    count_value = (count_value << 8) + count_4;

    return count_value;
}

void clearEncoderCount() {
    // Set encoder1's data register to 0
    // digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    PORTE &= 0xFB;  // Clear PE2
    // Write to DTR
    SPI.transfer(0x98);
    // Load data
    SPI.transfer(0x00);  // Highest order byte
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);  // lowest order byte
    // digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation
    PORTE |= 0x04;  // Set PE2

    delayMicroseconds(
        100);  // provides some breathing room between SPI conversations

    // Set encoder1's current data register to center
    // digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    PORTE &= 0xFB;  // Clear PE2
    SPI.transfer(0xE0);
    // digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation
    PORTE |= 0x04;  // Set PE2

    // Set encoder2's data register to 0
    digitalWrite(slaveSelectEnc2, LOW);  // Begin SPI conversation
    // Write to DTR
    SPI.transfer(0x98);
    // Load data
    SPI.transfer(0x00);  // Highest order byte
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);                   // lowest order byte
    digitalWrite(slaveSelectEnc2, HIGH);  // Terminate SPI conversation

    delayMicroseconds(
        100);  // provides some breathing room between SPI conversations

    // Set encoder2's current data register to center
    digitalWrite(slaveSelectEnc2, LOW);  // Begin SPI conversation
    SPI.transfer(0xE0);
    digitalWrite(slaveSelectEnc2, HIGH);  // Terminate SPI conversation

    // Set encoder3's data register to 0
    digitalWrite(slaveSelectEnc3, LOW);  // Begin SPI conversation
    // Write to DTR
    SPI.transfer(0x98);
    // Load data
    SPI.transfer(0x00);  // Highest order byte
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);                   // lowest order byte
    digitalWrite(slaveSelectEnc3, HIGH);  // Terminate SPI conversation

    delayMicroseconds(
        100);  // provides some breathing room between SPI conversations

    // Set encoder3's current data register to center
    digitalWrite(slaveSelectEnc3, LOW);  // Begin SPI conversation
    SPI.transfer(0xE0);
    digitalWrite(slaveSelectEnc3, HIGH);  // Terminate SPI conversation

    // Set encoder4's data register to 0
    digitalWrite(A3, LOW);  // Begin SPI conversation
    // Write to DTR
    SPI.transfer(0x98);
    // Load data
    SPI.transfer(0x00);  // Highest order byte
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);      // lowest order byte
    digitalWrite(A3, HIGH);  // Terminate SPI conversation

    delayMicroseconds(
        100);  // provides some breathing room between SPI conversations

    // Set encoder4's current data register to center
    digitalWrite(A3, LOW);  // Begin SPI conversation
    SPI.transfer(0xE0);
    digitalWrite(A3, HIGH);  // Terminate SPI conversation
}

// void setPwmFrequency(int pin, int divisor) {
//  // This code is for Arduino Mega
//  byte mode;
//  if(pin == 11 || pin == 12 || pin == 13) { // pins using TIMER1
//    switch(divisor) {
//      case 1: mode = 0x01; break;     //  31250 hz
//      case 8: mode = 0x02; break;     //  3906.25 hz
//      case 64: mode = 0x03; break;    //  488.28125 hz (default)
//      case 256: mode = 0x04; break;   //  122.0703125 hz
//      case 1024: mode = 0x05; break;  //  30.517578125 hz
//      default: return;
//    }
//    TCCR1B = TCCR1B & 0b11111000 | mode;
//  }
//  else if(pin == 9 || pin == 10) { // pins using TIMER2
//      switch(divisor) {
//      case 1: mode = 0x01; break;     // 31250 hz
//      case 8: mode = 0x02; break;     // 3926.25 hz
//      case 32: mode = 0x03; break;    // 976.5625 hz
//      case 64: mode = 0x04; break;    // 488.28125 hz (default)
//      case 128: mode = 0x05; break;   // 244.140625 hz
//      case 256: mode = 0x06; break;   // 122.0703125 hz
//      case 1024: mode = 0x07; break;  // 30.517578125 hz
//      default: return;
//    }
//    TCCR2B = TCCR2B & 0b11111000 | mode;
//  }
//   else if(pin == 2 || pin == 3 || pin == 5) { // pins using TIMER3
//      switch(divisor) {
//      case 1: mode = 0x01; break;     // 31250 hz
//      case 8: mode = 0x02; break;     // 3926.25 hz
//      case 64: mode = 0x03; break;    // 488.28125 hz (default)
//      case 128: mode = 0x04; break;   // 122.0703125 hz
//      case 1024: mode = 0x05; break;  // 30.517578125 hz
//      default: return;
//    }
//    TCCR3B = TCCR3B & 0b11111000 | mode;
//  }
//   else if(pin == 6 || pin == 7 || pin == 8) { // pins using TIMER4
//      switch(divisor) {
//      case 1: mode = 0x01; break;     // 31250 hz
//      case 8: mode = 0x02; break;     // 3926.25 hz
//      case 64: mode = 0x03; break;    // 488.28125 hz (default)
//      case 128: mode = 0x04; break;   // 244.140625 hz
//      case 1024: mode = 0x05; break;  // 30.517578125 hz
//      default: return;
//    }
//    TCCR4B = TCCR4B & 0b11111000 | mode;
//  }
//     else if(pin == 44 || pin == 45 || pin == 46) { // pins using TIMER5
//      switch(divisor) {
//      case 1: mode = 0x01; break;     // 31250 hz
//      case 8: mode = 0x02; break;     // 3926.25 hz
//      case 64: mode = 0x03; break;    // 488.28125 hz (default)
//      case 128: mode = 0x04; break;   // 244.140625 hz
//      case 1024: mode = 0x05; break;  // 30.517578125 hz
//      default: return;
//    }
//    TCCR5B = TCCR5B & 0b11111000 | mode;
//  }
//}
