/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

/**
 * UPLOAD THIS CODE TO THE SENDER
 */

#include <esp_now.h>
#include <WiFi.h>

/* 
 *  REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
 *  ADD OR DELETE LINES DEPENDING ON NUMBER OF RECEIVER BOARDS
 */
uint8_t broadcastAddress1[] = {0x50, 0x02, 0x91, 0xA1, 0x96, 0x6C};


//type struct with two integer variables
typedef struct test_struct {
  int x;
  int y;
} test_struct;

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
  Serial.begin(115200);

  //initialize device as wifi station
  WiFi.mode(WIFI_STA);

  //initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }


  //register callback function to be called when a message is sent
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
  test.x = random(0,20);
  test.y = random(0,20);

  //send the message - first argument is mac address, if you pass 0 then it sends the same message to all registered peers
  esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(test_struct));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
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
