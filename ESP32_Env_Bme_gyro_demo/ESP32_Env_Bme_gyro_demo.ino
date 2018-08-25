/*
  CMPS03 with arduino I2C example

  This will display a value of 0 - 359 for a full rotation of the compass.

  The SDA line is on analog pin 4 of the arduino and is connected to pin 3 of the CMPS03.
  The SCL line is on analog pin 5 of the arduino and is conected to pin 2 of the CMPS03.
  Both SDA and SCL are also connected to the +5v via a couple of 1k8 resistors.
  A switch to callibrate the CMPS03 can be connected between pin 6 of the CMPS03 and the ground.
*/

#include <Wire.h>
#include "Adafruit_BME280.h"
#include "WiFi.h"

#define I2C_SCL ((uint8_t)26)
#define I2C_SDA ((uint8_t)27)
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_ADD 0x76


namespace {
const uint8_t HEADING = 2;
const uint8_t ANGLE_8 = 1;
const uint8_t CMPS11_ADDRESS = 0x60;
const int WAIT_UNTIL_ACTION = 100;

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16;
int bearing;

Adafruit_BME280 bme((uint8_t)I2C_SDA, (uint8_t)I2C_SCL);

};

void setup() {
  bool status;
  Wire.begin(I2C_SDA, I2C_SCL);; //conects I2C
  status = bme.begin(BME280_ADD);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  //setup WiFi stuff
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
    
  Serial.begin(115200);
}

void loop() {
    // WiFi.scanNetworks will return the number of networks found
  //https://github.com/espressif/arduino-esp32/issues/758 (next two lines)
  WiFi.disconnect();
  WiFi.begin("lolllool","loooooooool"); //Can be a nonexistent network
  printNetworks(WiFi.scanNetworks());
    
  getHeading();
  getPitchRollAngle();
  Serial.print("Bearing: ");
  Serial.print(bearing);
  Serial.print("\tRoll: ");               // Display roll data
  Serial.print(roll, DEC);
  Serial.print("\tPitch: ");          // Display pitch data
  Serial.print(pitch, DEC);
  Serial.print("\tAngle full: ");     // Display 16 bit angle with decimal place
  Serial.print(angle16 / 10, DEC);
  Serial.print(".");
  Serial.print(angle16 % 10, DEC);
  Serial.print("\tAngle 8: ");        // Display 8bit angle
  Serial.println(angle8, DEC);
  printEnv();

  delay(500);
}

void getHeading() {
  byte highByte;
  byte lowByte;
  Wire.beginTransmission(CMPS11_ADDRESS);      //starts communication with cmps03
  Wire.write(HEADING);                         //Sends the register we wish to read
  Wire.endTransmission();
  Wire.requestFrom(CMPS11_ADDRESS, 2);        //requests high byte
  long wait = millis();

  while ((millis() - wait) < WAIT_UNTIL_ACTION) {
    if (Wire.available() == 2) {       //while there is a byte to receive
      highByte = Wire.read();           //reads the byte as an integer
      lowByte = Wire.read();
      bearing = ((highByte << 8) + lowByte) / 10;
      break;
    }
  }
  //typically < 6ms including a tons of print statements;
  //Serial.print("Total I2C round trip transaction time for Heading was: ");
  //Serial.print(millis() - wait); Serial.println(" ms");
}


void getPitchRollAngle() {

  Wire.beginTransmission(CMPS11_ADDRESS);      //starts communication with cmps03
  Wire.write(ANGLE_8);                         //Sends the register we wish to read
  Wire.requestFrom(CMPS11_ADDRESS, 5);        //requests high byte
  long wait = millis();
  while ((millis() - wait) < WAIT_UNTIL_ACTION) {
    if (Wire.available() == 5) {       //while there is a byte to receive
      // Request 5 bytes from the CMPS11
      Wire.requestFrom(CMPS11_ADDRESS, 5);
      while (Wire.available() < 5);       // Wait for all bytes to come back
      angle8 = Wire.read();               // Read back the 5 bytes
      high_byte = Wire.read();
      low_byte = Wire.read();
      pitch = Wire.read();
      roll = Wire.read();

      angle16 = high_byte;                 // Calculate 16 bit angle
      angle16 <<= 8;
      angle16 += low_byte;

      break;
    }
  }
  //typically < 12ms ...including tons of print statements
  //Serial.print("Total I2C round trip transaction time for Roll/Pitch/Angle was: ");
  //Serial.print(millis() - wait); Serial.println(" ms");
}

void printEnv() {
  Serial.print("Temperature: ");
  Serial.print(bme.readTemperature());
  Serial.print(" ℃");

  Serial.print("\tPressure: ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.print(" hPa");

  Serial.print("\tAltitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.print("m");

  Serial.print("\tHumidity: ");
  Serial.print(bme.readHumidity());
  Serial.println("%");
}

void printNetworks(int numNetworksFound){
  if (numNetworksFound == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(numNetworksFound);
        Serial.println(" networks found");
        for (int i = 0; i < numNetworksFound; ++i) {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
            delay(10);
        }
    }
    Serial.println("");
}




