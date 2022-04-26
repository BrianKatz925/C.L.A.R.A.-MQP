// Copyright 2020 Shoushan Chiang

#include <avr/io.h>

/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>

#include "Adafruit_MPR121.h"
// Beginning of Auto generated function prototypes by Atmel Studio
static inline void initTimers(void);
void initEncoders();
long readEncoder(int encoder);
void clearEncoderCount();
void receiveMotorCommand(int howMany);
void requestEncoderData();
void requestTouchState();
void initializeModule();
// End of Auto generated function prototypes by Atmel Studio

const int I2C_SLAVE_NUM = 10;

// Adafruit_MPR121 cap = Adafruit_MPR121();
// #define SET_FLAG(REGISTER,FLAG)     REGISTER|=(FLAG)
// #define CLEAR_FLAG(REGISTER,FLAG)   REGISTER&=~(FLAG)
const int MOTOR_ONE_DIR_PIN = 4;  // 12
const int MOTOR_ONE_PWM_PIN = 5;  // 13
const int MOTOR_TWO_DIR_PIN = 6;
const int MOTOR_TWO_PWM_PIN = 9;
const int MOTOR_THREE_DIR_PIN = 12;  // 10
const int MOTOR_THREE_PWM_PIN = 10;  // 5
// const int slaveSelectEnc1 = 13;
const int slaveSelectEnc2 = 8;
const int slaveSelectEnc3 = 7;

// MOTOR PROPERTIES
float gear_ratio = 150.0;
// for 1000:1 gear ratio, 986*12 = 11832 = 1 rotation,
// new motor purchased by Ruibo has 150:1 gear ratio
int encoder_ticks_per_rev = 7;
// NO OF HIGH PULSE PER ROTATION, THERE ARE 7 pairs of magnet poles on
// the encoder magnet, for module 8 (the one with double origami modules)
// somehow 12 PPR works much better.
int quad_count_mode_multiplier = 4;
// 4X COUNT MODE QUADRATURE ENCODER set to the LS7366R encoder chip
float encoder_ticks_to_angle =
    (360 / (float)(gear_ratio * encoder_ticks_per_rev));  // in degrees

// ORIGAMI MODULE PROPERTIES
double shaft_diameter = 7.0;
double pcb_to_shaft_offset = 11.0;
double max_module_length = 180.0;
// distance from PCB to top plate when origami is fully extended
// (natural length) measured using ruler or caliper in mm, 210
double min_module_length = 60.0;  // distance from PCB to top plate when origami
                                  // is fully compressed in mm, 60

double delta_t = 0.01;  // in seconds
double ticks = 0;
double ticks_counted = 0;
double ticks_two = 0;
double ticks_counted_two = 0;
double ticks_three = 0;
double ticks_counted_three = 0;

float motor_one_output = 0.0;
double mo = 0.0;
double mo2 = 0.0;
double mo3 = 0.0;
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
double des_length1 = 0;
double des_length2 = 0;
double des_length3 = 0;
byte motorCommand[6];  // previous is 3
byte encoderData[6];
double encoder[3];
byte ready = 1;
byte notready = 0;

/// NEWEST PID PARAMETERS
double Kp1 = 0.5, Ki1 = 0.005,
       Kd1 = 0.005;  // Kp1 = 1.5 for module with double origami
double Kp2 = 0.5, Ki2 = 0.005, Kd2 = 0.005;
double Kp3 = 0.5, Ki3 = 0.005, Kd3 = 0.005;

unsigned long loop_index = 0;
signed long encoder1count = 0;
signed long encoder2count = 0;
signed long encoder3count = 0;

bool timer5_flag = false;
bool switch_direction_buffer_state = false;
long timer5_count = 0;

int touch_sense_cap_val = 0;
int touch_sense_cap_val2 = 0;
bool touch_switch_read = HIGH;
byte touch_state[1];

PID m1pid(&encoder[0], &mo, &des_length1, Kp1, Ki1, Kd1, DIRECT);
PID m2pid(&encoder[1], &mo2, &des_length2, Kp2, Ki2, Kd2, DIRECT);
PID m3pid(&encoder[2], &mo3, &des_length3, Kp3, Ki3, Kd3, DIRECT);
int motor1_cur_val = 0;
int motor2_cur_val = 0;
int motor3_cur_val = 0;
double m1_cur_val = 0.0;
double m2_cur_val = 0.0;
double m3_cur_val = 0.0;
double m1_prev_val = 0.0;
double m2_prev_val = 0.0;
double m3_prev_val = 0.0;
double alpha = 0.1;

int count = 0;
byte motor_state = 0;

bool motor1_stall = false;
bool motor2_stall = false;
bool motor3_stall = false;
double encoder_offset = 0;
// use this offset if the module is initialize to be fully compressed
// without extending motion, the value is updated inside the initialize
// module function
int command_type = 1;

static inline void initTimers(void) {
    // Init timers for motor PWM using Fast PWM mode, use this instead of
    // analog to get changeable PWM frequency

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
}

void setup() {
    Wire.begin(I2C_SLAVE_NUM);
    Wire.onReceive(receiveMotorCommand);  // register event
    Wire.onRequest(requestEncoderData);
    // Wire.onRequest(requestTouchState);
    Serial1.begin(9600);
    // cap.begin(0x5A);

    // Configure PORTB pin 0 as an input, equivalent to
    // CLEAR_FLAG(DDRB,bit(0));
    PORTB |= (1 << PORTB0);  // Activate pull-ups in PORTB pin 0, equivalent to
    // SET_FLAG(PORTB,bit(0));

    pinMode(MOTOR_ONE_DIR_PIN, OUTPUT);
    pinMode(MOTOR_ONE_PWM_PIN, OUTPUT);
    pinMode(MOTOR_TWO_DIR_PIN, OUTPUT);
    pinMode(MOTOR_TWO_PWM_PIN, OUTPUT);
    pinMode(MOTOR_THREE_DIR_PIN, OUTPUT);
    pinMode(MOTOR_THREE_PWM_PIN, OUTPUT);
    m1pid.SetMode(AUTOMATIC);
    m1pid.SetSampleTime(2);
    m1pid.SetOutputLimits(-255, 255);
    m2pid.SetMode(AUTOMATIC);
    m2pid.SetSampleTime(2);
    m2pid.SetOutputLimits(-255, 255);
    m3pid.SetMode(AUTOMATIC);
    m3pid.SetSampleTime(2);
    m3pid.SetOutputLimits(-255, 255);
    initEncoders();
    Serial1.println("Encoders Initialized...");
    clearEncoderCount();
    Serial1.println("Encoders Cleared...");
    initTimers();
    // Serial1.println("Initiliazing module length");
    // initializeModule();
}

void loop() {
    //  encoder1count = readEncoder(1);
    //  encodercount = (double) encoder1count;
    //  encoder2count = readEncoder(2);
    //  encodercount2 = (double) encoder2count;
    //  encoder3count = readEncoder(3);
    //  encodercount3 = (double) encoder3count;

    if (command_type == 0) {
        Serial1.println("Initiliazing module length");
        initializeModule();
    }

    for (int i = 0; i < 3; i++) {
        encoder[i] = (double)readEncoder(i + 1) +
                     encoder_offset;  // motor position read by encoder
                                      // in double type

        // need to make encoderData to be 4 byte eventually
        encoderData[i * 2] = long(encoder[i]) >> 8;
        encoderData[i * 2 + 1] = long(encoder[i]) & 255;

        /* example how to make 4 byte data
           long lng = 200000L;
           byte payload[4];
           payload[0] = (byte) ((lng & 0xFF000000) >> 24 );
           payload[1] = (byte) ((lng & 0x00FF0000) >> 16 );
           payload[2] = (byte) ((lng & 0x0000FF00) >> 8  );
           payload[3] = (byte) ((lng & 0X000000FF)       );
        */
    }

    /*
      touch_sense_cap_val = cap.filteredData(0);
      touch_sense_cap_val2 = cap.filteredData(1);

      if (touch_sense_cap_val <= 165)
      {
      touch_state[0] = 1;
      }
      else
      {
      touch_state[0] = 0;
      }

      if (touch_sense_cap_val2 <= 165)
      {
      touch_state[1] = 1;
      }
      else
      {
      touch_state[1] = 0;
      }

    */

    //  encoderData[0] = encodercount;
    //  encoderData[1] = encodercount2;
    //  encoderData[2] = encodercount3;
    //  encoderData[3] = encodercount3;
    //  encoderData[4] = encodercount3;
    //  encoderData[5] = encodercount3;

    m1pid.Compute();
    m2pid.Compute();
    m3pid.Compute();

    if (mo < 0) {
        digitalWrite(MOTOR_ONE_DIR_PIN,
                     LOW);  // for new motors this should be LOW
    } else {
        digitalWrite(MOTOR_ONE_DIR_PIN, HIGH);
    }

    if (mo2 < 0) {
        digitalWrite(MOTOR_TWO_DIR_PIN, LOW);
    } else {
        digitalWrite(MOTOR_TWO_DIR_PIN, HIGH);
    }

    if (mo3 < 0) {
        digitalWrite(MOTOR_THREE_DIR_PIN, LOW);
    } else {
        digitalWrite(MOTOR_THREE_DIR_PIN, HIGH);
    }

    touch_switch_read =
        PINB & (1 << PINB0);  // read pin, equivalent to PINB&bit(0);
    if (touch_switch_read == 0) {
        touch_state[0] = 1;
    } else if (touch_switch_read == 1) {
        touch_state[0] = 0;
    }

    ////// MOTOR CONTROL USING PWM ANALOG WRITE
    //  analogWrite(MOTOR_ONE_PWM_PIN, abs(mo));
    //  analogWrite(MOTOR_TWO_PWM_PIN, abs(mo2));
    //  analogWrite(MOTOR_THREE_PWM_PIN, abs(mo3));

    ////// MOTOR CONTROL USING PWM WITHOUT ANALOG WRITE
    OCR3A = abs(mo);  // Send PWM duty cycle to register associate with pin
    // connected to motor 1
    OCR1A = abs(mo2);  // Send PWM duty cycle to register associate with pin
    // connected to motor 2
    OCR1B = abs(mo3);  // Send PWM duty cycle to register associate with pin
    // connected to motor 3

    ////// MOTOR CURRENT READING
    // for motor1 (A0), motor2 (A1), motor3 (A2)
    motor1_cur_val = analogRead(A0);
    motor2_cur_val = analogRead(A1);
    motor3_cur_val = analogRead(A2);

    m1_cur_val = alpha * motor1_cur_val + (1 - alpha) * m1_cur_val;
    m2_cur_val = alpha * motor2_cur_val + (1 - alpha) * m2_cur_val;
    m3_cur_val = alpha * motor3_cur_val + (1 - alpha) * m3_cur_val;

    /// /// This need to be commented otherwise the control won't work very
    /// well because the serial print slows down everything. 1 Serial print
    /// is okay if we want to check how good the PID control is.
    /*Serial1.print(des_length1);
      Serial1.print("\t");
      Serial1.print(des_length2);
      Serial1.print("\t");
      Serial1.println(des_length3);
      Serial1.print("\t");
    */

    /*Serial1.print(encoder[0]);
      Serial1.print("\t");
      Serial1.print(encoder[1]);
      Serial1.print("\t");
      Serial1.println(encoder[2]);
    */

    // Serial1.println(touch_sense_cap_val);
}

void initEncoders() {
    // Set slave selects as outputs
    // pinMode(slaveSelectEnc1, OUTPUT);
    DDRE |= 0x04;  // pinMode PE2 OUTPUT
    pinMode(slaveSelectEnc2, OUTPUT);
    pinMode(slaveSelectEnc3, OUTPUT);

    // Raise select pins
    // Communication begins when you drop the individual select signsl
    // digitalWrite(slaveSelectEnc1,HIGH);
    PORTE |= 0x04;  // Set PE2
    digitalWrite(slaveSelectEnc2, HIGH);
    digitalWrite(slaveSelectEnc3, HIGH);

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
    digitalWrite(slaveSelectEnc2, LOW);  // Begin SPI conversation
    SPI.transfer(0x88);                  // Write to MDR0
    SPI.transfer(0x03);  // x4 quadrature count mode, 4 byte counter mode is by
    // default on MDR1 register
    digitalWrite(slaveSelectEnc2, HIGH);  // Terminate SPI conversation

    // Initialize encoder 3
    //    Clock division factor: 0
    //    Negative index input
    //    free-running count mode
    //    x4 quatrature count mode (four counts per quadrature cycle)
    // NOTE: For more information on commands, see datasheet
    digitalWrite(slaveSelectEnc3, LOW);  // Begin SPI conversation
    SPI.transfer(0x88);                  // Write to MDR0
    SPI.transfer(0x03);  // x4 quadrature count mode, 4 byte counter mode is by
    // default on MDR1 register
    digitalWrite(slaveSelectEnc3, HIGH);  // Terminate SPI conversation
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
        count_4 = SPI.transfer(0x00);  // Read lowest order byte
        digitalWrite(slaveSelectEnc2,
                     HIGH);  // Terminate SPI conversation
    }

    else if (encoder == 3) {
        digitalWrite(slaveSelectEnc3, LOW);  // Begin SPI conversation
        SPI.transfer(0x60);                  // Request count
        count_1 = SPI.transfer(0x00);        // Read highest order byte
        count_2 = SPI.transfer(0x00);
        count_3 = SPI.transfer(0x00);
        count_4 = SPI.transfer(0x00);  // Read lowest order byte
        digitalWrite(slaveSelectEnc3,
                     HIGH);  // Terminate SPI conversation
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

    // Set encoder2's data register to 0
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

    // Set encoder2's current data register to center
    digitalWrite(slaveSelectEnc3, LOW);  // Begin SPI conversation
    SPI.transfer(0xE0);
    digitalWrite(slaveSelectEnc3, HIGH);  // Terminate SPI conversation
}

void receiveMotorCommand(int howMany) {
    /*
      int index = 0;
      while (Wire.available())
      {
      byte d = Wire.read();
      if (d != 255)
      {
      motorCommand[index] = d;
      //motorCommand[index] = Wire.read();
      }
      index = index + 1;
      }

      des_length1 = (motorCommand[0]<<8)+motorCommand[1];
      des_length2 = (motorCommand[2]<<8)+motorCommand[3];
      des_length3 = (motorCommand[4]<<8)+motorCommand[5];
    */

    int index = 0;
    while (Wire.available()) {
        byte d = Wire.read();
        if (index < 6) {
            motorCommand[index] = d;
        } else {
            command_type = d;
        }
        index = index + 1;
    }

    // Need to make this 4 byte eventually
    des_length1 = (motorCommand[0] << 8) + motorCommand[1];
    des_length2 = (motorCommand[2] << 8) + motorCommand[3];
    des_length3 = (motorCommand[4] << 8) + motorCommand[5];

    // Example how to make 4 byte variable
    // des_length1 = ((long)(motorCommand[0])<<24) +
    // ((long)(motorCommand[1])<<16)
    // + ((long)(motorCommand[2])<<8) + ((long)(motorCommand[3])) ;  //
    // motor command in 4 bytes
}

void requestEncoderData() { Wire.write(encoderData, 6); }

void requestTouchState() { Wire.write(touch_state, 1); }

void initializeModule() {
    motor1_stall = false;
    motor2_stall = false;
    motor3_stall = false;
    int threshold_value = 700;  // out of 1023
    int timeout_count = 0;
    digitalWrite(
        MOTOR_ONE_DIR_PIN,
        HIGH);  // for new motors LOW corresponds to clockwise rotation or
    // negative values of encoder, we want to start with
    // positive value so set to HIGH for new motors
    digitalWrite(MOTOR_TWO_DIR_PIN, HIGH);
    digitalWrite(MOTOR_THREE_DIR_PIN, HIGH);

    while (timeout_count < 10000 &&
           (motor1_stall == false || motor2_stall == false ||
            motor3_stall == false)) {
        ////// MOTOR CURRENT READING
        // for motor1 (A0), motor2 (A1), motor3 (A2)
        motor1_cur_val = analogRead(A0);
        motor2_cur_val = analogRead(A1);
        motor3_cur_val = analogRead(A2);

        m1_cur_val = alpha * motor1_cur_val + (1 - alpha) * m1_cur_val;
        m2_cur_val = alpha * motor2_cur_val + (1 - alpha) * m2_cur_val;
        m3_cur_val = alpha * motor3_cur_val + (1 - alpha) * m3_cur_val;

        for (int i = 0; i < 3; i++) {
            encoder[i] = (double)readEncoder(i + 1);
        }

        if (m1_cur_val > threshold_value) {
            motor1_stall = true;
        } else {
            motor1_stall = false;
        }

        if (m2_cur_val > threshold_value) {
            motor2_stall = true;
        } else {
            motor2_stall = false;
        }

        if (m3_cur_val > threshold_value) {
            motor3_stall = true;
        } else {
            motor3_stall = false;
        }

        if (motor1_stall == false) {
            OCR3A = 255;  // Send PWM duty cycle to register
                          // associate with pin connected to motor 1
        } else if (motor1_stall = true) {
            OCR3A = 0;  // Send PWM duty cycle to register associate
                        // with pin connected to motor 1
        }

        if (motor2_stall == false) {
            OCR1A = 255;  // Send PWM duty cycle to register
                          // associate with pin connected to motor 2
        } else if (motor2_stall = true) {
            OCR1A = 0;  // Send PWM duty cycle to register associate
                        // with pin connected to motor 2
        }

        if (motor3_stall == false) {
            OCR1B = 255;  // Send PWM duty cycle to register
                          // associate with pin connected to motor 3
        } else if (motor3_stall = true) {
            OCR1B = 0;  // Send PWM duty cycle to register associate
                        // with pin connected to motor 3
        }

        timeout_count = timeout_count + 1;
        // Serial1.print(motor_state);
        // Serial1.print("\t\t");

        /*Serial1.print(encoder[0]);
          Serial1.print("\t");
          Serial1.print(encoder[1]);
          Serial1.print("\t");
          Serial1.println(encoder[2]);
          Serial1.println(timeout_count);
        */
    }

    clearEncoderCount();
    Serial1.println("Encoders Cleared...");
    int loop_count = 0;
    des_length1 = -0.01 * encoder_ticks_per_rev * gear_ratio *
                  quad_count_mode_multiplier;  // plus for old motor,  -3.32
                                               // is obtained by
    // dividing the cable length (110 - 7 - 30) mm
    // by circumference of spool (pi*d) where d =
    // 7mm, 30 mm is the collapsed length of the
    // origami module, for longer module
    // (260-7-(pi*3.5))/(pi*7) = 10.5
    des_length2 = des_length1;
    des_length3 = des_length1;

    while (loop_count < 2000) {
        for (int i = 0; i < 3; i++) {
            encoder[i] = (double)readEncoder(i + 1);
        }

        m1pid.Compute();
        m2pid.Compute();
        m3pid.Compute();

        if (mo < 0) {
            digitalWrite(MOTOR_ONE_DIR_PIN,
                         LOW);  // for new motors this should be LOW
        } else {
            digitalWrite(MOTOR_ONE_DIR_PIN, HIGH);
        }

        if (mo2 < 0) {
            digitalWrite(MOTOR_TWO_DIR_PIN, LOW);
        } else {
            digitalWrite(MOTOR_TWO_DIR_PIN, HIGH);
        }

        if (mo3 < 0) {
            digitalWrite(MOTOR_THREE_DIR_PIN, LOW);
        } else {
            digitalWrite(MOTOR_THREE_DIR_PIN, HIGH);
        }

        // OCR3A = abs(mo);    // Send PWM duty cycle to register
        // associate with pin connected to motor 1 OCR1A = abs(mo2); //
        // Send PWM duty cycle to register associate with pin connected
        // to motor 2 OCR1B = abs(mo3);   // Send PWM duty cycle to
        // register associate with pin connected to motor 3

        loop_count = loop_count + 1;
        // Serial1.print(loop_count);
        // Serial1.print("\t\t");
        // Serial1.print(encoder[0]);
        // Serial1.print("\t");
        // Serial1.print(encoder[1]);
        // Serial1.print("\t");
        // Serial1.println(encoder[2]);
    }
    clearEncoderCount();
    Serial1.println("Module initialized");
    /*
      des_length1 = 7.8*encoder_ticks_per_rev*gear_ratio*quad_count_mode
      ultiplier;		//
      minus for old motor, module will start with some certain short length
      des_length2 = des_length1;
      des_length3 = des_length1;
    */

    // FIND ENCODER OFFSET VALUE TO COMPENSATE FOR INTIAL STATE OF MODULE
    // BEING FULLY COMPRESSED
    encoder_offset = (max_module_length - min_module_length) /
                     (shaft_diameter * pi) *
                     (encoder_ticks_per_rev * gear_ratio *
                      quad_count_mode_multiplier);  // multiplier of two at
                                                    // the denominator is
    // because we are using X4 quadrature mode
    des_length1 =
        encoder_offset;  // set desired lengths to be the encoder offset
    des_length2 = des_length1;
    des_length3 = des_length1;

    /*
      Serial1.print(loop_count);
      Serial1.print("\t\t");
      Serial1.print(encoder[0]);
      Serial1.print("\t");
      Serial1.print(encoder[1]);
      Serial1.print("\t");
      Serial1.println(encoder[2]);
    */

    command_type = 1;
}