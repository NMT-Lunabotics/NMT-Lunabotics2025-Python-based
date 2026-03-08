#include <SoftwareSerial.h>
#define RX_PIN 10
SoftwareSerial ibusSerial(RX_PIN, -1);

bool debug_mode = false;


//Joystick centers and safe zones, along with the joysticks min and max output values.
//MotorX, MotorY, actuatorX, actuatorY
int16_t centers[4] = { 1640, 1615, 1624, 1622 };       
int16_t softZones[4] = { 50, 250, 50, 50 };           
int16_t maxJoyValues[4] = { 1971, 1976, 2000, 2000 };  
int16_t minJoyValues[4] = { 1000, 1040, 1087, 1064 };  


void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT);
  ibusSerial.begin(115200);
}

void loop() {
  int16_t* joystick = readIbus();
  if (joystick != nullptr) {
    for (int i = 0; i < 4; i++) {
      if(i==0) Serial.print("Motor Joystick X: ");
      if(i==1) Serial.print("Motor Joystick Y: ");
      if(i==2) Serial.print("Actuator Joystick X: ");
      if(i==3) Serial.print("Actuator Joystick Y: ");
      Serial.print(joystick[i]);
      if (i < 3) Serial.print(",");
    }
    Serial.println();
  }
  delay(20);
}

int16_t* readIbus() {
  static int16_t channels[6];
  static int16_t joystick[4];
  //Create buffer for our data stream
  static uint8_t buffer[32];
  static int idx = 0;

  while (ibusSerial.available()) {
    uint8_t val = ibusSerial.read();
    //Check if buffer index matches the data index for the first and last index to unsure we received a valid data stream.
    if (idx == 0 && val != 0x20) continue;
    if (idx == 1 && val != 0x40) {
      idx = 0;
      continue;
    }
    buffer[idx++] = val;
    //Once we recive whole stream convert data values between 1000-2000
    if (idx == 32) {
      idx = 0;
      uint16_t chksum = 0xFFFF;
      for (int i = 0; i < 30; i++) chksum -= buffer[i];
      uint16_t pktChksum = buffer[30] | (buffer[31] << 8);
      if (chksum == pktChksum) {
        for (int i = 0; i < 6; i++) {
          int pos = 2 + (i * 2);
          channels[i] = buffer[pos] | (buffer[pos + 1] << 8);
        }
        for (int i = 0; i < 4; i++) {
          int16_t val;
          switch (i) {
            case 0: val = channels[3]; break;
            case 1: val = channels[2]; break;
            case 2: val = channels[0]; break;
            case 3: val = channels[1]; break;
          }
          if (debug_mode == false) {
            if (val >= centers[i] - softZones[i] && val <= centers[i] + softZones[i]) {
              joystick[i] = 0;  
            } else if (val < centers[i] - softZones[i]) {
              joystick[i] = map(val, minJoyValues[i], centers[i] - softZones[i], -255, 0);
              if (joystick[i] < -255) joystick[i] = -255;  
            } else {                                       
              joystick[i] = map(val, centers[i] + softZones[i], maxJoyValues[i], 0, 255);
              if (joystick[i] > 255) joystick[i] = 255;  
            }
          } else joystick[i] = val;
        }
        return joystick;
      }
    }
  }
  return nullptr;
}