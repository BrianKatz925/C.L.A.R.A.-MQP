#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h> //include Wire.h library

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xA8, 0x03, 0x2A, 0x6B, 0x70, 0xD8}; //RED/BLACK ROBOT
//uint8_t broadcastAddress[] = {0x84, 0x0D, 0x8E, 0xE6, 0x7C, 0x98}; //BLUE/BLACK ROBOT


// Structure example to send data
// Must match the receiver structure
typedef struct test_struct {
  // two strings for restoring data from A and B 两个字符串分别用于存储A、B两端传来的数据:
  int M1 = 0;
  int M2 = 0;
  int M3 = 0;
  int dataArray[7] = {0,0,0,0,0,0,1};
} test_struct;

String data_cache = "";

String device_B_String="";

double no_of_max_turn = 8;
double gear_ratio = 150;
double length1 = 0;// length1~3:0~100
double length2 = 0;
double length3 = 0;
double encoder_ticks_per_rev = 14;
int flag = 1;


// Create a struct_message called myData
test_struct test;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}
 
void setup() {
  // serial initializing:
  Serial.begin(115200);
  // WIre communication begin
  Wire.begin();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

  
void loop() {
  byte error, address = 10; //variable for error and I2C address
  
  // 读取从计算机传入的数据，并通过serial1发送个设备B:
  if(Serial.available()>0)
  {
    if(Serial.peek()!='\n')   //在没接收到回车换行的条件下
    {
      device_B_String+=(char)Serial.read();   //输入整数
      data_cache = device_B_String;   //输入整数
    }
    else
    {  //这段代码实现从缓冲区读取数据，并将数据发送到计算机显示和软串口发送；
      Serial.read();
      Serial.print("you said:");
      Serial.println(device_B_String);

      int i = device_B_String.toInt();
      double temp = i;
      temp = no_of_max_turn * temp * encoder_ticks_per_rev * gear_ratio/100.0;
      device_B_String = "";
      
      if(flag == 1)
      {
        test.dataArray[0] = (int)temp >> 8;
        test.dataArray[1] = (int)temp & 255;
        Serial.print("M1: ");
        Serial.println(i);
        Serial.println(temp);
        Serial.println((int)temp);
        flag = 2;
      }
      else if(flag == 2)
      {
        test.dataArray[2] = (int)temp >> 8;
        test.dataArray[3] = (int)temp & 255;
        Serial.print("M2: ");
        Serial.println(i);
        Serial.println(temp);
        Serial.println((int)temp);
        flag = 3;
      }
      else if(flag == 3)
      {
        test.dataArray[4] = (int)temp >> 8;
        test.dataArray[5] = (int)temp & 255;
        Serial.print("M3: ");
        Serial.println(i);
        Serial.println(temp);
        Serial.println((int)temp);
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
        test.M1 = data_cache.toInt();
        Serial.print("M1:");
        Serial.println(test.M1);
        flag = 5;
        data_cache = "";
      }
      else if(flag == 5)
      {
        Serial.read();
        test.M2 = data_cache.toInt();
        Serial.print("M2:");
        Serial.println(test.M2);
        flag = 6;
        data_cache = "";
      }
      else if(flag == 6)
      {
        Serial.read();
        test.M3 = data_cache.toInt();
        Serial.print("M3:");
        Serial.println(test.M3);
        flag = 1;
        data_cache = "";
        esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
        test.M1 = 0;
        test.M2 = 0;
        test.M3 = 0;
      }
       
      delay(1000); // wait 5 seconds for the next I2C scan

    }
  } 
}
