#include "helpers.hpp"
// #include <Servo.h>
//--------------- Debug settings ---------------
// MotorX, MotorY, actuatorX, actuatorY
int16_t centers[4] = { 1640, 1615, 1624, 1622 };       // Center of each joystick
int16_t softZones[4] = { 50, 250, 50, 50 };            // Joy stick Drift ranges
int16_t maxJoyValues[4] = { 1971, 1976, 2000, 2000 };  // Joy stick max range
int16_t minJoyValues[4] = { 1000, 1040, 1087, 1064 };  // Joy sick min ranges

// List of components flags to enable/disable for testing
bool error_leds_enabled = false;
bool motors_enabled = false;
bool backet_actuator_enabled = false;
bool arm_actuators_enabled = false;
bool servo_motor_enabled = false;

// List of faults to disable
bool serial_communication_timeout_fault = false;
bool component_timeout_faults = false;

// Debug mode flag
bool debug_mode = false; 
bool sensor_output = 0; //1: IMU

#define ledCR 48
#define ledCG 50
#define ledCB 52
int ledCRV = 255;
int ledCGV = 255;
int ledCBV = 255;

//--------------- SYSTEM VARIABLES ---------------

unsigned long current_time = 0;
float last_message_time = 0;
bool receiving_message = false;
int serial_index = 0;
int expected_length = -1;
const int SERIAL_BUFFER_SIZE = 128;
byte serial_buffer[SERIAL_BUFFER_SIZE];


void setup() {
  delay(5);
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.flush();
  pinMode(ledCR, OUTPUT);
  pinMode(ledCG, OUTPUT);
  pinMode(ledCB, OUTPUT);
  Serial.println("Arduino system_control.ino started.");
}

void loop() {
  analogWrite(ledCR, ledCRV);
  analogWrite(ledCG, ledCGV);
  analogWrite(ledCB, ledCBV);
  current_time = millis();
  // Read serial and process messages while being Non-blocking 
  if (Serial2.available()) {
  int16_t *joystick = processIbus();
  ledCRV=joystick[1]; 
  ledCGV=joystick[2];
  ledCBV=joystick[3]; 
  }
  else{
  int ledCRV = 255;
  int ledCGV = 255;
  int ledCBV = 255;
  }
  while (Serial.available() > 0) {
    byte b = Serial.read();
    if (!receiving_message) {
        if (b == 0x02) {
            receiving_message = true;
            serial_index = 0;
            expected_length = -1;
        }
    } else {
        if (expected_length == -1) {
            expected_length = b;
            if (expected_length <= 0 || expected_length > SERIAL_BUFFER_SIZE) {
                receiving_message = false; // invalid length
            }
        } else {
            serial_buffer[serial_index++] = b;
            if (serial_index == expected_length + 1) { // +1 for end byte
                if (serial_buffer[serial_index-1] == 0x03) {
                    processMessage(serial_buffer, expected_length);
                } else {
                    Serial.println("End byte not found");
                }
                receiving_message = false;
            } else if (serial_index >= SERIAL_BUFFER_SIZE) {
                // overflow protection
                receiving_message = false;
            }
        }
    }
}
}

// Read serial communication from autonomy computer and set system variables to output.
void processMessage(byte *data, int length) {
  current_time = millis();
  char type = data[0];
  if (debug_mode) {
    Serial.print("Received message of type: ");
    Serial.println(type);
    Serial.print("Length: ");
    Serial.println(length);
  }
  bool cmd_triggered = true;
  switch (type) {
    case 'L':
      {                   
        ledCRV=data[1]; 
        ledCGV=data[2];
        ledCBV=data[3]; 
        break;
      }
    default:
      Serial.println("Unknown message type");
      cmd_triggered=false;
      break;
  }
  last_message_time=current_time;
}

// Read serial communication from RC controller and set system variables to output
int16_t *processIbus() {
  static int16_t channels[6];
  static int16_t joystick[4];
  // Create buffer for our data stream
  static uint8_t buffer[32];
  static int idx = 0;

  if (Serial2.available()) {
    uint8_t val = Serial2.read();
    // Check if buffer index matches the data index for the first and last index to unsure we received a valid data stream.
    if (idx == 0 && val != 0x20)
      return nullptr;
    if (idx == 1 && val != 0x40) {
      idx = 0;
      return nullptr;
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