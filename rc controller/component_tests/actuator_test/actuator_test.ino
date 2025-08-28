#include <Wire.h>

// Address of MD04 driver
#define AL_I2C_ADDRESS 0x5A // B0
#define AR_I2C_ADDRESS 0x59 // B4
#define AB_I2C_ADDRESS 0x58 // B2 
#define ADDRESS 0x5A


#define CMDBYTE             0x00 // Command byte
#define SPEEDBYTE           0x02 // Byte to write to speed register
void setup(){
  Serial.begin(9600);                             
  Wire.begin();// Start I2C connection
}

void loop(){
    sendData(SPEEDBYTE, 0);// Send speed data
    sendData(CMDBYTE, 2); // Send Run Direction Command     
}

void sendData(byte commandRegister, byte value){// Send data through I2C communication
  Wire.beginTransmission(ADDRESS);// Start transmission of data to MD04 motor driver address
    Wire.write(commandRegister);// Select used register
    Wire.write(value);// Send data to register 
  Wire.endTransmission();   
}
