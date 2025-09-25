//This code is pushed to an external adruino, then by connecting it to the system control arduino the external adruino will take rc controller inputs and send serial

//--------------- RC controller ---------------

// MotorX, MotorY, actuatorX, actuatorY
int16_t centers[4] = { 1640, 1615, 1624, 1622 };       // Center of each joystick
int16_t softZones[4] = { 50, 250, 50, 50 };            // Joy stick Drift ranges
int16_t maxJoyValues[4] = { 1971, 1976, 2000, 2000 };  // Joy stick max range
int16_t minJoyValues[4] = { 1000, 1040, 1087, 1064 };  // Joy sick min ranges



void setup() {
  delay(5);
  //pinMode(RX_PIN, INPUT);
  //ibusSerial.begin(115200);
  Serial2.begin(115200);
  Serial.begin(115200);
  Serial.flush();
}

void loop() {
    if(Serial2.available()){
  int16_t *joystick = processIbus();
  if (joystick != nullptr) {
    //Motor commands
    uint8_t joystickData[4] = { joystick[2], joystick[3] };
    sendSerialCommand('M', joystickData, sizeof(joystickData) / sizeof(joystickData[0]));
    //Actuator commands
    //uint8_t joystickData[4] = { -1,-1,-1,-1, joystick[0], joystick[1] };
    //sendSerialCommand('A', joystickData, sizeof(joystickData) / sizeof(joystickData[0]));
  }
}
}

// Read serial communication from RC controller and set system variables to output
int16_t *processIbus() {
  static int16_t channels[6];
  static int16_t joystick[4];
  // Create buffer for our data stream
  static uint8_t buffer[32];
  static int idx = 0;

  while (Serial2.available()) {
    uint8_t val = Serial2.read();
    // Check if buffer index matches the data index for the first and last index to unsure we received a valid data stream.
    if (idx == 0 && val != 0x20)
      continue;
    if (idx == 1 && val != 0x40) {
      idx = 0;
      continue;
    }
    buffer[idx++] = val;
    // Once we recive whole stream convert data values from between 1000-2000 to 0-255
    if (idx == 32) {
      idx = 0;
      uint16_t chksum = 0xFFFF;
      for (int i = 0; i < 30; i++)
        chksum -= buffer[i];
      uint16_t pktChksum = buffer[30] | (buffer[31] << 8);
      if (chksum == pktChksum) {
        for (int i = 0; i < 6; i++) {
          int pos = 2 + (i * 2);
          channels[i] = buffer[pos] | (buffer[pos + 1] << 8);
        }
        for (int i = 0; i < 4; i++) {
          int16_t val;
          switch (i) {
            case 0:
              val = channels[3];
              break;
            case 1:
              val = channels[2];
              break;
            case 2:
              val = channels[0];
              break;
            case 3:
              val = channels[1];
              break;
          }
          if (val >= centers[i] - softZones[i] && val <= centers[i] + softZones[i]) {
            joystick[i] = 0;
          } else if (val < centers[i] - softZones[i]) {
            joystick[i] = map(val, minJoyValues[i], centers[i] - softZones[i], 255, 129);
            if (joystick[i] > 255) joystick[i] = 255;
          } else {
            joystick[i] = map(val, centers[i] + softZones[i], maxJoyValues[i], 0, 127);
            if (joystick[i] > 127) joystick[i] = 127;
          }
        }
        return joystick;
      }
    }
  }
  return nullptr;
}

void sendSerialCommand(char command, uint8_t* data, size_t dataLen) {
  const uint8_t startByte = 0x02; // STX
  const uint8_t endByte   = 0x03; // ETX

  uint8_t length = dataLen + 1; // command byte + data bytes

  // Make sure Serial is ready (important on Leonardo/Micro)
  while (!Serial) { ; }

  Serial.write(startByte);       // start byte
  Serial.write(length);          // length byte
  Serial.write((uint8_t)command); // command byte

  for (size_t i = 0; i < dataLen; i++) {
      Serial.write(data[i]);    // data bytes
  }

  Serial.write(endByte);         // end byte
  Serial.flush();                // ensure all bytes are sent
}

