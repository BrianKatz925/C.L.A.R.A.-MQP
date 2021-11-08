#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include <esp_now.h>
#include <WiFi.h>

//Structure example to receive data
//Must match the sender structure
typedef struct test_struct {
  int M1 = 0;
  int M2 = 0;
  int M3 = 0;
  int dataArray[7] = {0,0,0,0,0,0,1};
} test_struct;

String device_B_String="";

//Create a struct_message called myData
test_struct myData;


//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("device_B_String: ");
  for(int j = 0; j < 7; j++)
        {
          Serial.print(myData.dataArray[j]);
          Serial.print(",");
        }
  Serial.print("\n");
  Serial.println();
  
  int M1 = myData.M1;
  if(M1>255){M1 = 255;}
  else if(M1<-255){M1 = -255;}
  else{M1 = M1;}
  int M2 = myData.M2;
  if(M2>255){M2 = 255;}
  else if(M2<-255){M2 = -255;}
  else{M2 = M2;}
  int M3 = myData.M3;
  if(M3>255){M3 = 255;}
  else if(M3<-255){M3 = -255;}
  else{M3 = M3;}
  
  Serial.print("M1: ");
  Serial.println(M1);
  Serial.print("M2: ");
  Serial.println(M2);
  Serial.print("M3: ");
  Serial.println(M3);
}

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);


void setup() {
  Serial.begin(115200);

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  int8_t i=0;
  myMotor1->run(FORWARD);
  myMotor1->setSpeed(150);
  // turn on motor
  myMotor1->run(RELEASE);

    myMotor2->run(FORWARD);
  myMotor2->setSpeed(150);
  // turn on motor
  myMotor2->run(RELEASE);

    myMotor3->run(FORWARD);
  myMotor3->setSpeed(150);
  // turn on motor
  myMotor3->run(RELEASE);

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  Wire.begin(); // Wire communication begin

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  byte error, address = 10; //variable for error and I2C address
  
  
  Wire.beginTransmission(address); 
  for (int i=0; i<7; i++)
        {
          Wire.write(myData.dataArray[i]);  //data bytes are queued in local buffer
        }             
  Wire.endTransmission();     
  
  delay(10);

  if(myData.M1<0)
  {
    myMotor1->run(BACKWARD);
    myMotor1->setSpeed(abs(myData.M1));  
  }
  else
  {
    myMotor1->run(FORWARD);
    myMotor1->setSpeed(myData.M1);
  }

  if(myData.M2<0)
  {
    myMotor2->run(BACKWARD);
    myMotor2->setSpeed(abs(myData.M2));  
  }
  else
  {
    myMotor2->run(FORWARD);
    myMotor2->setSpeed(myData.M2);
  }

  if(myData.M3<0)
  {
    myMotor3->run(BACKWARD);
    myMotor3->setSpeed(abs(myData.M3));  
  }
  else
  {
    myMotor3->run(FORWARD);
    myMotor3->setSpeed(myData.M3);
  }
  
  delay(10);

}
