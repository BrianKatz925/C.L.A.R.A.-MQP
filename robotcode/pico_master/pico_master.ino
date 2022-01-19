/**
 * Uploaded to Mainboard - Handles I2C communication
 * Current setup - Mainboard requests data from motor driver, motor driver returns data
 * Need to do the opposite - Motor driver should send data and mainboard receives it - ESP32 doesn't have onreceive function in arduino core so we need to poll manually
**/


#include <Wire.h> //include Wire.h library
//#include <esp_now.h> //ESP-Wifi comms
#include <WiFi.h> 


void setup()
{
  Wire.begin(); // I2C communication begin
  Serial.begin(9600); // The baudrate of Serial monitor is set in 9600 - lower baudrates work best with motor driver modules
  while (!Serial); // Waiting for Serial Monitor to initialize
  Serial.println("\nI2C Scanner"); 

  //Set device as a Wi-Fi Station
//  WiFi.mode(WIFI_STA);
//
//  //Init ESP-NOW
//  if (esp_now_init() != ESP_OK) {
//    Serial.println("Error initializing ESP-NOW");
//    return;
//  }
//
//  // Once ESPNow is successfully Init, we will register for recv CB to
//  // get recv packer info
//  esp_now_register_recv_cb(OnDataRecv);

  findDevices() ; //run once on startup to verify SAMIs connected 

}


char buf[3]; //preset character array with 2 bytes of information 


void loop() {
  
}


void sendMsg(int address, int message){
  Wire.beginTransmission(address);
  Wire.write(message);
  Wire.endTransmission();
}

void requestData(int address, int numBytes){
  Wire.requestFrom(address, numBytes); //create a request from an individual motor driver board for 2 bytes of information
  if (Wire.available() > 1) {
    for (int i = 0; i < 3; i++)
    {
      buf[i] = Wire.read();  //read along i2C line if data is available //arduino buf[8] = Wire.read();  //arduino
    }

    //print out received data
    Serial.print("Data: ");
    Serial.println(buf);
  }
}







/****************************************
 * I2C helper functions
 ************************************/

/**
 * Finds all available I2C devices on the current bus - can be used for troubleshooting
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
 * ESP-NOW helper functions
 
 
//Structure example to receive data
//Must match the sender structure
typedef struct data_struct {
  int wifiData;
} data_struct;

//Create a struct_message called myData
data_struct myData;

/**
 * callback function that will be executed when data is received - currently calls master to request data along I2C line
 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData)); //copy content of incomingdata variable into mydata variable
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Data Received: ");
  Serial.println(myData.wifiData);
  Serial.println();

  //when receiving a WiFi command from a TinyPico, request data on I2C bus
  if(myData.wifiData == 1) {
     readI2C();
  }

}*/
