/**
   Uploaded to Mainboard - Handles I2C communication between smart motor driver boards, WiFi communication between external esp
**/


#include <Wire.h> //include Wire.h library
#include <esp_now.h> //ESP-Wifi comms
#include <WiFi.h>
#include <Math.h>


/**
    Wheel Addresses
    1,2,3 - Drive Wheels
    4,5,6 - Cable Motors
    7 - Lead Screw Motor
*/

/**********************************************************************************************************************

                                                    VARIABLES + DEFINITIONS


 **********************************************************************************************************************/


//ESP-NOW Variables
uint8_t broadcastAddress1[] = {0x98, 0xCD, 0xAC, 0x61, 0x58, 0xAC}; //replace with the MAC Address of your ESP

typedef struct data_struct { //data struct to send to the receiver/external ESP containing address, current sensor, and encoder data
  // data types must match data struct sent from external ESP
  int smdAddress;
  int currentData;
  int encoderData;
} data_struct;

typedef struct data_struct_rec { //data struct to receive wifi commands from the external ESP  - must match data struct sent from external ESP
  String wifiData;
} data_struct_rec;

data_struct test; //create instance of sending struct to be populated with data
data_struct_rec myData; //create instance of receiving struct


//data variables from smart motor drivers
int count = 0; //encoder count
char current = 0; //current reading
const int vRef = 3.3; //reference logic level voltage
const int senseResistor = 0.5; //current sense resistor in Ohms
byte enc1, enc2; //encoder variables
const int encTicksPerRev = 12; //encoder ticks per revolution
const float motorGearRatio = 298.00; //gear ratio on high strength N20 motors

//cable variables
int cablelenraw = 0; //unused
float cablelenadj = 0; //unused
//cable length and encoder count variables
float l1 = 0;
float l2 = 0;
float l3 = 0;
int c1, c2, c3;

const float drumdiameter = 0.25; //Diameter of the winch drums for the cable motors - inches i guessed
const float origlength = 5.0; //original length of the Yoshimura module

const float r = 7 ; //length of module in inches
const float L0 = 1 ;//shortest length of module in inches

float s, theta, phi; //arc length, bending angle, bending directions - from soft robotics lab paper



/**********************************************************************************************************************

                                                         SETUP + LOOP


 **********************************************************************************************************************/

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

  //run once on startup to verify SAMIs connected
  findDevices();

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

void loop() {
  //nothing really.... this is all event based
}


/**********************************************************************************************************************

                                                        I2C FUNCTIONS


 **********************************************************************************************************************/

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
   Reads encoder & current data from a singular motor driver board, stores them, and sends them to the external ESP for debugging
   @param address - the hexadecimal I2C address of the motor driver you are requesting data from
   @param numBytes - the number of bytes you are requesting from the motor driver board over I2C
*/
void requestData(int address, int numBytes) {
  Wire.requestFrom(address, numBytes, true);//create a request from an individual motor driver board for given number of bytes

  // if we receive the expected number of bytes
  if (Wire.available() == numBytes) {
    Serial.print("data recieved from: ");
    Serial.println(address);

    // read data in one by one and store them into separate variables
    enc1 = Wire.read();
    enc2 = Wire.read();
    count = enc1;
    count = (count << 8) | enc2; //put the two bytes back together to get encoder count
    current = Wire.read(); //current sent second
    int motorrpm = Wire.read();

    //print out received data
    Serial.print("Encoder Count: ");
    int readcount = count;
    if (readcount > 32768) { //prevent overflow of integer type from prolonged use of encoder
      readcount = 65535 - readcount;
      readcount *= -1;
    }
    Serial.print(readcount);
    Serial.print('\t');

    float readcurrent = current / 510.0; //(255 * senseResistor) ; //get the ADC reading

    Serial.print("Current: ");
    Serial.print(readcurrent);
    Serial.print('\t');

    //set values into the send struct to send to the external ESP
    test.smdAddress = address;
    test.currentData = readcurrent * 100; //floats had issues sending over ESP-NOW, multiply by 100 and send as an int
    test.encoderData = readcount;
    Serial.print("motor rpm:  ");
    Serial.println(motorrpm);

    // if the address is one of the cable motors (4,5,6), calculate their cable lengths with encoder data
    if (address == 0x04) {
      c1 = readcount; //encoder count variable for cable 1
      l1 = calcCablelen(c1);
    } else if (address == 0x05) {
      c2 = readcount; //encoder count variable for cable 2
      l2 = calcCablelen(c2);
    }
    else if (address == 0x06) {
      c3 = readcount; //encoder count variable for cable 3
      l3 = calcCablelen(c3);

    }

    // Serial.println(test.encoderData);

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

/**
   Sends a message to the provided smart motor driver address via I2C
   @param address - the hexadecimal address of the motor driver board you want to send a message to
   @param message - the character message you want to write to
*/
void sendMsg(int address, char message) {
  Wire.beginTransmission(address); //open up I2C bus
  Wire.write(message); //write a message
  int error = Wire.endTransmission();
  if (error != 0) { //if we receive an error, print it out
    Serial.println("Error sending command: ");
    Serial.println(error);
  }
  else {
    // Serial.println("Message sent");

  }
}


/**********************************************************************************************************************

                                                    ESP-NOW WIFI HELPER FUNCTIONS


 **********************************************************************************************************************/

/**
    Callback function to be executed when WiFi data is sent to external ESP
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
  // Serial.print(" send status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


/**
   Callback function to be executed when WiFi data is received from external ESP
   Handles controller command data and commands motor driver boards to move or send data based on input
   @param mac - the mac address of the board sending the data
   @param incomingData - the data to be copied into the myData variable - the instance of the receiving struct
   @param len the number of bytes received
*/
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData)); //copy content of incomingdata variable into mydata variable

  //  Serial.print("Bytes received: ");
  //  Serial.println(len);
  //  Serial.print("Data Received: ");
  //  Serial.println(myData.wifiData);
  //  Serial.println();

  // convert data into an integer to figure out what command was sent from the controller
  int commanddata = myData.wifiData.toInt();

  /*********************
     Controller Responses
   *********************/

  /**
     button pad - drive the lead screw and request data
  */

  if (commanddata == 1) { //Y button - lead screw up
    driveleadscrew(1); /// fix
    Serial.println("lead screw up");
  }
  else if (commanddata == 4) { //A button - lead screw down
    driveleadscrew(2); /// fix
    Serial.println("lead screw down");

  }
  else if (commanddata == 2) { //X button - send data back from all motors
    Serial.println("requesting Data");
    requestData(0x01, 4);
    requestData(0x02, 4);
    requestData(0x03, 4);
    requestData(0x04, 4);
    requestData(0x05, 4);
    requestData(0x06, 4);
    requestData(0x07, 4);
    invCableKin(l1, l2, l3); //do the inv kinematics i guess
  }


  /**
     D-Pad - control the cable motors
  */

  else if (commanddata == 10) { //dpad = 0 - home
    Serial.println("homing cables");
    drivecables(0, 0, 0);
    drive(0);
    driveleadscrew(0);
    //this will be something based off of current sensors later i imagine
  }
  else if (commanddata == 11) { //dpad = 1 - all cables down
    Serial.println("all cables down");
    //drivecables(12, 12, 12);
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
    //drivecables(13, 13, 13);
  }
  else if (commanddata == 18) { //reverse cable 1
    Serial.println("cable 1 up");
    drivecables(13, 0, 0);
  }
  else if (commanddata == 19) { //reverse cable 2
    Serial.println("cable 2 up");
    drivecables(0, 13, 0);
  }
  else if (commanddata == 20) { //reverse cable 3
    Serial.println("cable 3 up");
    drivecables(0, 0, 13);
  }


  /**
     wheel driving - drive wheels and lead screw based on current sensing
  */

  else if (commanddata == 8) { //right bumper - drive forward fast
    Serial.println("drive forward");
    drive(8);
  }
  else if (commanddata == 9) { //left bumper - drive backward fast
    drive(9);
    Serial.println("drive backward");
  }
  else if (commanddata == 21) { //right bumper - drive forward fast
    Serial.println("drive forward");
    drive(10);
  }
  else if (commanddata == 22) { //right bumper - drive forward fast
    Serial.println("drive forward");
    drive(11);
  }
  else if (commanddata == 23) {
    Serial.println("close lead screw w/ current");
    driveleadscrew(3);
  }
  else if (commanddata == 24) {
    Serial.println("open lead screw w/ current");
    driveleadscrew(4);
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


/**********************************************************************************************************************

                                                    SMART MOTOR DRIVER DRIVE FUNCTIONS


 **********************************************************************************************************************/

/**
   drives drive wheels at the speed given, will work on sending an actual ramped speed later
   @param speed - the integer speed that the drive wheels will rotate
*/
void drive(int speed) {
  sendMsg(0x01,  speed);
  sendMsg(0x02,  speed);
  sendMsg(0x03,  speed);
}

/**
   Drives cables at the given speed for each cable
   @param speed1 - the integer speed of cable 1 (motor 4)
   @param speed2 - the integer speed of cable 2 (motor 5)
   @param speed3 - the integer speed of cable 3 (motor 6)
*/
void drivecables(int speed1, int speed2, int speed3) {
  sendMsg(0x04,  speed1); //cable 1
  sendMsg(0x05,  speed2); //cable 2
  sendMsg(0x06,  speed3); //cable 3
}

/**
   Drives lead screw given speed
   @param speed - the integer speed of the lead screw
*/
void driveleadscrew(int speed) {
  if (speed == 0) {
    sendMsg(0x07, 0); //manually stop lead screw
  } else {
    sendMsg(0x07, speed);
  } //im so gonna have to change this later not even gonna cap
}



/**
   Calculates the length of all cables using encoder counts
   @param enccounts - the integer amount of quadrature encoder ticks
   @return float cablelen - the length of the cable
*/
float calcCablelen(int enccounts) {
  //Serial.println(enccounts);
  int16_t rotations1 = abs(enccounts) / encTicksPerRev; //calculate number of rotations of the motor shaft from the encoder wheel
  float rotations = rotations1 / motorGearRatio; //calculate actual number of rotations from motor gearbox
  //Serial.println(rotations);
  float deltacablelen = rotations * M_PI * drumdiameter; //calculate the change in cable length from circumference of drum diameter and the amount of rotations
  float cablelen = origlength - deltacablelen; //calculate final cable length
  return cablelen;
}

/**
   Calculates the inverse kinematics (bending angle - theta, arc length - S, bending direction - phi) of the cables given the cable lengths
   @param l1 - float length of cable 1
   @param l2 - float length of cable 2
   @param l3 - float length of cable 3
*/
void invCableKin (float l1, float l2, float l3) {
  // calc S, theta, and phi
  Serial.print("L1, L2, L3, respectively: ");
  Serial.print(l1);
  Serial.print('\t');
  Serial.print(l2);
  Serial.print('\t');
  Serial.println(l3);


  s = (3 * L0 + l1 + l2 + l3) / 3;
  Serial.print("S is: ");
  Serial.print(s);
  Serial.print('\t');
  theta = 2 * sqrt((3 * (l1 * l1) - l1 * l2 - l1 * l3 - l2 * l3) / (3 * r));
  Serial.print("theta is: ");
  Serial.print(theta);
  Serial.print('\t');
  phi = atan((sqrt(3) * (l3 - l2)) / (l2 + l3 - 2 * l1));

  Serial.print("phi is: ");
  Serial.println(phi);
}
