/**
 * UPLOAD THIS CODE TO THE SENDER
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

/* 
 *  REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
 *  ADD OR DELETE LINES DEPENDING ON NUMBER OF RECEIVER BOARDS
 */
uint8_t broadcastAddress1[] = {0x50, 0x02, 0x91, 0xA1, 0x96, 0x6C};


//type struct with two integer variables
typedef struct test_struct {
  // M1,2,3 = linear expansion motors - require encoders
  // M4,5,6 = drive motors
  // M7 = radial expansion motor - require encoders
  // each motor that we will receive encoder data from needs two values in the data array
  
  int M1 = 0; 
  int M2 = 0;
  int M3 = 0;
  int M4 = 0;
  int M5 = 0;
  int M6 = 0;
  int M7 = 0;
  //                  M1  M2  M3  M7  ex
  int dataArray[9] = {0,0,0,0,0,0,0,0,1};

} test_struct;


String data_cache = "";
String device_B_String="";

double no_of_max_turn = 8;
double gear_ratio = 150;
double length1 = 0;// length1~3:0~100
double length2 = 0;
double length3 = 0;
double encoder_ticks_per_rev = 14; // to change depending on our encoder ticks per revolution
int flag = 1;

test_struct test; //store variable values

// callback when data is sent
// executed when data is sent, prints if message was successfully delivered to know if board received message
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Begin Wire communication
  Wire.begin(); 

  //initialize device as wifi station
  WiFi.mode(WIFI_STA);

  //initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  //register callback function to be called when a message is sent
  //get status of sent packet
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  byte error, address = 10; //variable for error and I2C address

  if(Serial.available() > 0) //if the serial monitor is open and values can be input
  {
    if(Serial.peek() != '\n'){ //user typed something into serial input
      device_B_String+=(char)Serial.read();   //add string to a cache
      data_cache = device_B_String;   
    }
    else { //if you press the enter button to submit a string
      Serial.read();
      Serial.print("you said:");
      Serial.println(device_B_String);

      int input = device_B_String.toInt();
      double motorCommand = input;
      motorCommand = no_of_max_turn * motorCommand * encoder_ticks_per_rev * gear_ratio/100.0; 
      device_B_String = "";
      
      if(flag == 1)
      {
        test.dataArray[0] = (int)motorCommand >> 8;
        test.dataArray[1] = (int)motorCommand & 255;
        Serial.print("M1: ");
        Serial.println(input);
        Serial.println(motorCommand);
        Serial.println((int)motorCommand);
        flag = 2;
      }
      else if(flag == 2)
      {
        test.dataArray[2] = (int)motorCommand >> 8;
        test.dataArray[3] = (int)motorCommand & 255;
        Serial.print("M2: ");
        Serial.println(input);
        Serial.println(motorCommand);
        Serial.println((int)motorCommand);
        flag = 3;
      }
      else if(flag == 3)
      {
        test.dataArray[4] = (int)motorCommand >> 8;
        test.dataArray[5] = (int)motorCommand & 255;
        Serial.print("M3: ");
        Serial.println(input);
        Serial.println(motorCommand);
        Serial.println((int)motorCommand);
        flag = 4;
        for(int j = 0; j < 7; j++)
        {
          Serial.print(test.dataArray[j]);
          Serial.print(",");
        }
        Serial.print("\n");        
      }
       else if(flag == 4)
      {
        Serial.read();
        test.M4 = data_cache.toInt();
        Serial.print("M4:");
        Serial.println(test.M4);
        flag = 5;
        data_cache = "";
      }
      else if(flag == 5)
      {
        Serial.read();
        test.M5 = data_cache.toInt();
        Serial.print("M5:");
        Serial.println(test.M5);
        flag = 6;
        data_cache = "";
      }
      else if(flag == 6)
      {
        Serial.read();
        test.M6 = data_cache.toInt();
        Serial.print("M6:");
        Serial.println(test.M6);
        flag = 7;
        data_cache = "";
      }
      else if (flag == 7) {
        test.dataArray[6] = (int)motorCommand >> 8;
        test.dataArray[7] = (int)motorCommand & 255;
        Serial.print("M7: ");
        Serial.println(input);
        Serial.println(motorCommand);
        Serial.println((int)motorCommand);
        
        esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));

         if (result == ESP_OK) {
          Serial.println("Sent with success");
        }
        else {
          Serial.println("Error sending the data");
        }
        test.M1 = 0;
        test.M2 = 0;
        test.M3 = 0;
        test.M7 = 0;  
      }
      delay(1000); // wait 1 seconds for the next I2C scan
    }
  }
}


/**
 * to send data to inidivudal boards
 * esp_err_t result1 = esp_now_send(
  broadcastAddress1, 
  (uint8_t *) &test,
  sizeof(test_struct));
   
if (result1 == ESP_OK) {
  Serial.println("Sent with success");
}
else {
  Serial.println("Error sending the data");
}

 */
