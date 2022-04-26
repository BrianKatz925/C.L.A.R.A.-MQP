

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


typedef struct data_struct_rec { //data struct to send wifi commands to the mainboard  - must match data struct mainboard is expecting
  int s;
  int theta;
  int phi;
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

int currInput = 1; //variable to keep track if input is being input as K, S, or Phi

//inverse kinematics parameters
int s = 0;
int theta = 0;
int phi = 0;


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
bool invalidinput = false;
int supper = 4.0; //inches
int slower = 0; //inches
int thetaupper = 90.0; //degrees
int thetalower = 0.00; //degrees
int phiupper = 360.0 ; //degrees
int philower = 0.0 ; //degrees
 
void loop() {
  if (Serial.available() > 0) {
    if (Serial.peek() != '\n') //if we press enter in the serial monitor and sent data
    {
      deviceBData += (char) Serial.read(); //add read string into a data cache
    }
    else { //end of a line
      Serial.read();
      //now need to interpret deviceBData

      //sequentially store integer values and increment counter
      if (currInput == 1) {
        Serial.println("You said S = " + deviceBData);
        s = deviceBData.toInt();
        currInput++;
      }
      else if (currInput == 2) {
        Serial.println("You said Theta = " + deviceBData);
        theta = deviceBData.toInt();
        currInput++;
      }
      else if (currInput == 3) {
        Serial.println("You said Phi = " + deviceBData);
        phi = deviceBData.toInt();
        currInput++;
      }
      else { //send the input data, and reset the flag for more input
        invalidinput = false;
        //sends the actual data
        if (!(s<=supper && s>= slower)){
          Serial.println(" S is invalid. valid parameters are between 3.6 and 6.0 ");
          invalidinput = true;
        }
        if (!(theta<=thetaupper && theta>= thetalower)){
          Serial.println(" theta is invalid. valid parameters are between 0 and 90 ");
          invalidinput = true;
        }
        if (!(phi<=phiupper && phi>= philower)){
          Serial.println(" phi is invalid. valid parameters are between 0 and 360 ");
          invalidinput = true;
        }

        if (!invalidinput){
        test.s = s; //convert data to an integer
        test.theta = theta;
        test.phi = phi;

        //send the message - first argument is mac address, if you pass 0 then it sends the same message to all registered peers
        esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(data_struct_rec));
        if (result == ESP_OK) {
          //Serial.println("Sent with success");
        }
        else {
          //Serial.println("Error sending the data");
        }}
        else{
          Serial.println("invalid inputs. no message sent");
        }
        currInput = 1;
      }
      deviceBData = ""; //reset to prevent concatenation of data
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
