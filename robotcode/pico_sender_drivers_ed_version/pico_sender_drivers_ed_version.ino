

/**
   Uploaded to external ESP Sender - Handles WiFi communication to Mainboard and prints along Serial for debugging
*/

#include <esp_now.h>
#include <WiFi.h>

/**********************************************************************************************************************

                                                    VARIABLES + DEFINITIONS


 **********************************************************************************************************************/
//ESP-NOW Variables
uint8_t broadcastAddress1[] = {0x24, 0xA1, 0x60, 0x75, 0xB8, 0xE0}; //replace with the MAC Address of your ESP


typedef struct data_struct_rec { //data struct to receive wifi commands from the external ESP  - must match data struct sent from mainboard
  String wifiData;
} data_struct_rec;

typedef struct data_struct { //data struct to receive from the mainboard containing address, current sensor, and encoder data
  // data types must match data struct that mainboard is expecting
  int smdAddress;
  int currentData;
  int encoderData;
} data_struct;

data_struct_rec test; //create instance of sending struct to be populated with data
data_struct receiveData; //create instance of receiving struct 

String deviceBData = ""; //variable for inputting commands into the serial monitor and storing the input

String newData = "";

char deviceBdata;
char input[43];


/**********************************************************************************************************************

                                                         SETUP + LOOP


 **********************************************************************************************************************/
 
void setup() {
  Serial.begin(9600);

  //initialize device as wifi station
  WiFi.mode(WIFI_STA);

  //initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //register callback function to be called when a message is sent
  esp_now_register_send_cb(OnDataSent);

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // register peer
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if (Serial.available() > 0) {
    if (Serial.peek() != '\n') //if we press enter in the serial monitor and sent data
    {
      deviceBData += (char) Serial.read(); //add read string into a data cache
    }
    else { //end of aline
      Serial.read();
      //now need to interpret deviceBData
      Serial.print("You said: ");
      newData = interpretData(deviceBData);
      Serial.println(newData);

      //sends the actual data
      test.wifiData = newData; //convert data to an integer
      //send the message - first argument is mac address, if you pass 0 then it sends the same message to all registered peers
      esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(data_struct_rec));
      if (result == ESP_OK) {
        //Serial.println("Sent with success");
      }
      else {
        //Serial.println("Error sending the data");
      }
      deviceBData = "";
    }
  }

  delay(5);
}


/**********************************************************************************************************************

                                                    ESP-NOW WIFI HELPER FUNCTIONS


 **********************************************************************************************************************/

/**
    Callback function to be executed when WiFi data is sent to mainboard
    prints if message was successfully delivered to know if board received message
    @param mac_addr - the mac address of the board that data is being sent to
    @param status - the status of the transaction - success or fail
*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  //Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


/**
   Callback function to be executed when WiFi data is received from mainboard
   Prints received smart motor driver address, current sensor and encoder data
   @param mac - the mac address of the board sending the data
   @param incomingData - the data to be copied into the myData variable - the instance of the receiving struct
   @param len the number of bytes received
*/
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&receiveData, incomingData, sizeof(receiveData)); //copy content of incomingdata variable into mydata variable

  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Smart Motor Driver Address: ");
  Serial.println(receiveData.smdAddress);
  Serial.print("Current Sensor Reading: ");
  Serial.println(receiveData.currentData);
  Serial.print("Encoder Reading: ");
  Serial.println(receiveData.encoderData);
  Serial.println("");
}


/**********************************************************************************************************************

                                                    CONTROLLER FEEDBACK FUNCTIONS


 **********************************************************************************************************************/


/**
 * Function to interpret input from Logitech gamepad from registers and return a command as a String to send to the mainboard
 * @param data - String output from controller 
 */
String interpretData(String data) {
  data.toCharArray(input, 43);
  //Serial.println(data);
  
  //A button is index 0

  //make D pad into integer 0-7
  String dpad = "";
  int dpadval = 0;
  for (int i = 0; i <= 3; i++) {
    dpadval *= 2;
    if (data[i] == '1') dpadval++;
  }
  
  //check buttons
  if (data[5] == '1') { //x button - send data
    return "2";
  }

  if (data[8] == '1' && data[7] == '1') { //left bumper and down button - run the lead screw till current dunks on it
   // return "23"; 
  }
  if (data[8] == '1' && data[4] == '1') { //left bumper and down button - run the lead screw till current dunks on it
    //return "24"; 
  }
  if (data[4] == '1') { //y button - lead screw up
    return "4";
  }

  //second profile 
  if (data[6] == '1' && dpadval == 2) { //B button with dpad
    return "18";
  }
  if (data[6] == '1' && dpadval == 4) { //B button w dpad
    return "19";
  }
  if (data[6] == '1' && dpadval == 6) { //B button w dpad
    return "20";
  }
  if (data[6] == '1' && data[8] == '1') { //special drive mode
   // return "21";
  }
  if (data[6] == '1' && data[9] == '1') { //special drive mode
    //return "22";
  }

//  if (data[6]=='1'){ //just B button... for lead screw testing..... 
//    return "25";
//  }
//  

  if (data[7] == '1') { //A button - lead screw down
    return "1";
  }
  if (data[8] == '1') { //right bumper
    return "8";
  }
  if (data[9] == '1') { //left bumper
    return "9";
  }
  if (dpadval == 0) { //dpad 0 = home
    return "10";
  }
  if (dpadval == 1) { //dpad 1 = all down
    ///return "11";
  }
  if (dpadval == 2) { //cable 1 down
    return "24";
  }
  if (dpadval == 3) { //cable 1 +2 down
    //return "13";
  }
  if (dpadval == 4) { //cable 2 down
    return "25";
  }
  if (dpadval == 5) { //cable 2 + 3 down
    //return "15";
  }
  if (dpadval == 6) { //cable 3 down
    return "26";
  } if (dpadval == 7) { //dpad 7 - all up
    //return "17";
  }

  //  if (rjoyval !=128){  // < 130 && rjoyval > 125) { //stick has been moved
  //    if (rjoyval < 10) {
  //      return "10";
  //    }
  //    else {
  //      return String(rjoyval);
  //    }
  //  }
  else {
    return "0";
  }

  //
  //make right joystick int 0-255 (128 is nothing)
  //  String rjoy1 = "";
  //  int rjoyval = 0;
  //  for (int i = 34; i <= 42; i++) {
  //    rjoyval *= 2;
  //    if (data[i] == '1') rjoyval++;
  //  }
  //Serial.print("rjoy: ");
  //Serial.println(rjoyval);
  //Serial.print("dpad: ");
  //Serial.println(dpad);
}
