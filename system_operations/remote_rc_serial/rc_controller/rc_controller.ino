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
  Serial1.begin(9600);
  Serial.flush();
  Serial.println("RC controller initialized");
}

void loop() {
  // Process IBUS data for joystick and return all data.
  uint8_t test[3] = {0,255,0};
  sendSerialCommand('L', test, sizeof(test));
  //int16_t *joystick = processIbus();
    //if (joystick != nullptr) {
    //  uint8_t joystickData[3] = { joystick[1], joystick[2], joystick[3] }; 
    //  sendSerialCommand('L', joystickData, sizeof(joystickData));
    //}
}


// Send a single message
void sendSerialCommand(char command, uint8_t* data, size_t dataLen) {
  const uint8_t startByte = 0x02; // STX
  const uint8_t endByte   = 0x03; // ETX

  uint8_t buf[64];   // max size for USB CDC packet
  size_t idx = 0;

  buf[idx++] = startByte;
  buf[idx++] = dataLen + 1; // length (command + data)
  buf[idx++] = command;

  for (size_t i = 0; i < dataLen; i++) {
    buf[idx++] = data[i];
  }

  buf[idx++] = endByte;
  Serial1.write(buf, idx); 
  Serial1.flush();
  //Serial.write(buf, idx);   // one USB transfer
  //Serial.flush();           // block until transmitted
  //delay(1);
}