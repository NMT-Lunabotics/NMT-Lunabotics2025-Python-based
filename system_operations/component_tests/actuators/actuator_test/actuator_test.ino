#include <Wire.h>

int act1PotPin = A1;
int act1Pot=0;

#define ACT1_POT_PIN A1
#define ACT1_PWM_PIN 9
#define ACT1_DIR_PIN 8

// Debug mode flag
bool debug_mode = true;

//Addresses of MD04 driver: 0x5A=0xB0, 0x59=0xB4, 0x58=0xB2 
#define ADDRESS 0x5E//0x5E

const byte CMDBYTE   = 0x00; // Command register
const byte STATUS    = 0x01; // Status register (read)
const byte SPEEDBYTE = 0x02; // Speed register
const byte ACCEL     = 0x03; // Acceleration register
const byte TEMP      = 0x04; // Temperature (read)
const byte CURRENT   = 0x05; // Motor current (read)
const byte UNUSED    = 0x06; // Always 0
const byte VERSION   = 0x07; // Software revision

void setup(){
  Serial.begin(115200);                             
  Wire.begin();// Start I2C connection
  delay(1000);
  if(debug_mode==true) logAddresses();

  Serial.println(readData(0x28));

  sendData(0x34,0x09);
  delay(1000);
Serial.println(readData(0x28));


  

  //sendData(SPEEDBYTE, 255);// Send speed data
  //sendData(CMDBYTE, 1); // Send Run Direction Command 
}
void loop(){  
}

int actMax=1023;
//actMin=


void sendData(byte commandRegister, byte value){  // Send data through I2C communication
  Wire.beginTransmission(ADDRESS);                // Start transmission of data to MD04 motor driver address
  Wire.write(commandRegister);                    // Select used register
  Wire.write(value);                              // Send data to register 
  Wire.endTransmission();                         // End I2C communication
}

byte readData(byte commandRegister){              // Read data through I2C communication
  Wire.beginTransmission(ADDRESS);                // Start transmission of data to MD04 motor driver address
  Wire.write(commandRegister);                    // Select used register
  Wire.endTransmission();                         // End I2C communication
  Wire.requestFrom(ADDRESS, 1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

void logRegisters() {   //Log all data/information returned from the MD04 driver registers
  Serial.print("CMD=");       Serial.print(readData(CMDBYTE));
  Serial.print("STATUS=");   Serial.print(readData(STATUS));
  Serial.print("SPEED=");    Serial.print(readData(SPEEDBYTE));
  Serial.print("ACCEL=");    Serial.print(readData(ACCEL));
  Serial.print("TEMP=");     Serial.print(readData(TEMP));
  Serial.print("CURRENT=");  Serial.print(readData(CURRENT));
  Serial.print("UNUSED=");   Serial.print(readData(UNUSED));
  Serial.print("VERSION=");  Serial.println(readData(VERSION));
}

void logAddresses(){    //Loop through all MD04 driver addresses and log active one.
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) Serial.println("No I2C devices found\n");
}

void logMD04Mode() {
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x27); // Mode register
  Wire.endTransmission();

  Wire.requestFrom(ADDRESS, 1); // Request 1 byte
  if (Wire.available()) {
    byte mode = Wire.read();
    Serial.print("MD04 current mode: ");
    Serial.print(mode, DEC);
    Serial.print(" - ");
    switch (mode) {
      case 0 ... 7: Serial.println("I2C mode"); break;
      case 8: Serial.println("Analog 0-2.5-5V mode"); break;
      case 9: Serial.println("Analog 0-5V mode"); break;
      case 10: Serial.println("RC mode with timeout"); break;
      case 11: Serial.println("RC mode no timeout"); break;
      case 12: Serial.println("Serial mode"); break;
      default: Serial.println("Unknown mode"); break;
    }
  } else {
    Serial.println("No response from MD04!");
  }
}