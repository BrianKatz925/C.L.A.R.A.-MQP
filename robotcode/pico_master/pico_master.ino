/**
   Uploaded to Mainboard - Handles I2C communication
   Current setup - Mainboard requests data from motor driver, motor driver returns data
   Need to do the opposite - Motor driver should send data and mainboard receives it - ESP32 doesn't have onreceive function in arduino core so we need to poll manually
**/


#include <Wire.h> //include Wire.h libra
#include <esp_now.h> //ESP-Wifi comms
#include <WiFi.h>

/*
    REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
    ADD OR DELETE LINES DEPENDING ON NUMBER OF RECEIVER BOARDS
*/
uint8_t broadcastAddress1[] = {0x98, 0xCD, 0xAC, 0x61, 0x58, 0xAC}; //replace with ESP

String deviceBData = "";


//type struct with two integer variables
typedef struct data_struct {
  int smdAddress;
  int currentData;
  int encoderData;
} data_struct;

data_struct test; //store variable values

// callback when data is sent
// executed when data is sent, prints if message was successfully delivered to know if board received message
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


void setup()
{
  Wire.begin(); // I2C communication begin
  Serial.begin(9600); // The baudrate of Serial monitor is set in 9600 - lower baudrates work best with motor driver modules
  while (!Serial); // Waiting for Serial Monitor to initialize
  Serial.println("\nI2C Scanner");

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  findDevices() ; //run once on startup to verify SAMIs connected

  //register callback function to be called when a message is sent
  esp_now_register_send_cb(OnDataSent);

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


char buf[3]; //preset character array with 2 bytes of information
void loop() {
    //nothing really.... this is all event based 

}


//drives at the speed given, will work on sending an actual ramped speed later
void drive(int speed) {

  sendMsg(0x01,  speed);
  sendMsg(0x02,  speed);
  sendMsg(0x03,  speed);
}
void drivecables(int speed1, int speed2, int speed3) {
  sendMsg(0x04,  speed1); //cable 1
  sendMsg(0x05,  speed2); //cable 2
  sendMsg(0x06,  speed3); //cable 3
}
void driveleadscrew(int speed) {
  if (speed == 0){
    sendMsg(0x07,0);
  }else{
  sendMsg(0x07, speed);} //im so gonna have to change this later not even gonna cap
}

void sendMsg(int address, char message) {
  Wire.beginTransmission(address);
  Wire.write(message);
  int error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("Error sending command: ");
    Serial.println(error);
  }
  else {
    Serial.println("Message sent");

  }
}


//reading back data from the smart motor drivers 
int count = 0;
char current = 0;
int vRef = 3.3;
int senseResistor = 0.5 ;
byte enc1,enc2;
void requestData(int address, int numBytes) {
  Serial.println("data is requested bitch");
  Wire.requestFrom(address, numBytes, true); //create a request from an individual motor driver board for 2 bytes of information
  if (Wire.available() == 3) {
    Serial.println("three bytes recieved");
    enc1= Wire.read();
    enc2 = Wire.read();
    count = enc1;
    count = (count<<8)| enc2; //put the two bytes back together
    current = Wire.read(); //current sent second


    //print out received data
    Serial.print("Encoder: ");
    int readcount = count;
    if (readcount > 32768) { //gotta undo the negatives 
      readcount = 65535 - readcount;
      readcount *= -1;
    }
    Serial.print(readcount);
    Serial.print('\t');

    float readcurrent = current / 510.0; //(255*senseResistor) ; //get the ADC reading

    Serial.print("Current: ");
    Serial.println(readcurrent);

    //code to send the data here
    test.smdAddress = address;
    test.currentData = readcurrent*100;
    test.encoderData = readcount;
    Serial.print("test sructs   ");
    Serial.print(test.currentData);
    Serial.println(test.encoderData);
    //send the message - first argument is mac address, if you pass 0 then it sends the same message to all registered peers
    esp_err_t result = esp_now_send(0, (uint8_t *) &test, sizeof(data_struct));
    if (result == ESP_OK) {
      //Serial.println("Sent with success");
    }
    else {
      //Serial.println("Error sending the data");
    }

  }
}


/****************************************
   I2C helper functions
 ************************************/

/**
   Finds all available I2C devices on the current bus - can be used for troubleshooting
*/
void findDevices() {
  byte error, address; //variable for error and I2C address
  int nDevices; //number of devices found on I2C bus

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address); //begin transmission with each possible address
    error = Wire.endTransmission(); //returns 0 for success, 1,2,3,4 for other errors


    //if we receive a successful transaction then print out the device's address in hex format
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}



/**
   ESP-NOW helper functions
   callback function that will be executed when data is received - currently calls master to request data along I2C line
*///Structure example to receive data
//Must match the sender structure
typedef struct data_struct_rec {
  String wifiData;
} data_struct_rec;

//Create a struct_message called myData
data_struct_rec myData;


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData)); //copy content of incomingdata variable into mydata variable
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Data Received: ");
  Serial.println(myData.wifiData);
  Serial.println();

  int commanddata = myData.wifiData.toInt();
  //button pad
  if (commanddata == 1) { //Y button - lead screw up
    driveleadscrew(1); /// fix
    Serial.println("lead screw up");

  }
  else if (commanddata == 4) { //A button - lead screw down
    driveleadscrew(2); /// fix
    Serial.println("lead screw down");

  }
  else if (commanddata == 2) { //x button - send data back
    Serial.println("requesting Data");
    //requestData(0x01, 3);
    requestData(0x02, 2);
   // requestData(0x03, 2);
//    requestData(0x04, 2);
//    requestData(0x05, 2);
//    requestData(0x06, 2);
//    requestData(0x07, 2);

  }


  //dpad shit
  else if (commanddata == 10) { //dpad = 0 - home
    Serial.println("homing cables");
    drivecables(0, 0, 0);
    drive(0);
    driveleadscrew(0);
    //this will be something based off of current sensors later i imagine
  }
  else if (commanddata == 11) { //dpad = 1 - all cables down
    Serial.println("all cables down");
    drivecables(12, 12, 12);
  }
  else if (commanddata == 12) { //dpad = 2 - cable 1 down
    Serial.println("cable 1 down");
    drivecables(12, 0, 0);
  }
  else if (commanddata == 13) { //dpad = 3 - cable 1 and 2 down
    Serial.println("cable 1 and 2 down");
    drivecables(12, 12, 0);
  }
  else if (commanddata == 14) { //dpad = 4 - cable 2 down
    Serial.println("cable 2 down ");
    drivecables(0, 12, 0);
  }
  else if (commanddata == 15) { //dpad = 5 - cable 2 and 3 down
    Serial.println("cables 2 and 3 down");
    drivecables(0, 12, 12);
  }
  else if (commanddata == 16) { //dpad = 6 - cable 3 down
    Serial.println("cable 3 down");
    drivecables(0, 0, 12);
  }
  else if (commanddata == 17) { //dpad = 7 - all cables up
    Serial.println("all cables up");
    drivecables(13, 13, 13);
  }



  //wheel driving
  else if (commanddata == 8) { //right bumper - drive forward
    Serial.println("drive forward");
    drive(8);
  }
  else if (commanddata == 9) { //left bumper - drive backward
    drive(9);
    Serial.println("drive backward");
  }


  //brake motors
  else if (commanddata == 0) { //brake all motors/do nothing
    drive(0);
    drivecables(0, 0, 0);
  }
  else if (commanddata == 256) { //brake all motors
    drive(0);
    drivecables(0, 0, 0);
  }

}
