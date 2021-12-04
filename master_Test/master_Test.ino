

#include <Wire.h> //include Wire.h library



void setup()
{
  Wire.begin(); // Wire communication begin
  Serial.begin(9600); // The baudrate of Serial monitor is set in 9600
  while (!Serial); // Waiting for Serial Monitor
  Serial.println("\nI2C Scanner");

}
String data = "";
char buf[2] ; //;
void loop() {

  Wire.requestFrom(0x01, 2);
  if ( Wire.available() > 1) {
    for (int i = 0; i < 2; i++)
    {
      buf[i] = Wire.read();  //arduino buf[8] = Wire.read();  //arduino
    }
    Serial.print("Data: ");
    Serial.println(buf);
  }
 

  data = "";







  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    //    // the Write.endTransmisstion to see if
    //    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

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

  delay(1000); // wait 5 seconds for the next I2C scan*/
}
