/**
 * Uploaded to Mainboard - Handles I2C communication
 * Current setup - Mainboard requests data from motor driver, motor driver returns data
 * Need to do the opposite - Motor driver should send data and mainboard receives it - ESP32 doesn't have onreceive function in arduino core so we need to poll manually
**/


#include <Wire.h> //include Wire.h library


void setup()
{
  Wire.begin(); // Wire communication begin
  Serial.begin(9600); // The baudrate of Serial monitor is set in 9600 - lower baudrates work best with motor driver modules
  while (!Serial); // Waiting for Serial Monitor to initialize
  Serial.println("\nI2C Scanner"); 

}

String data = "";
char buf[2]; //preset character array with 2 bytes of information 
void loop() {

  //since we are only requesting 2 bytes, we should expect to receive only 2 bytes of information
  Wire.requestFrom(0x01, 2); //create a request from an individual motor driver board for 2 bytes of information
  if ( Wire.available() > 1) {
    for (int i = 0; i < 2; i++)
    {
      buf[i] = Wire.read();  //read along i2C line if data is available //arduino buf[8] = Wire.read();  //arduino
    }

    //print out received data
    Serial.print("Data: ");
    Serial.println(buf);
  }
 
  data = "";
  
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

  delay(1000); // wait 1 seconds for the next I2C scan
}
