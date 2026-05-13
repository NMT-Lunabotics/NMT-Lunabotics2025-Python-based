/*
 * head_pan.ino
 *
 * Minimal serial-controlled servo sketch inspired by system_control.ino.
 * Supports the same framed serial protocol and the 'S' command to set
 * the servo angle. Designed for Arduino UNO with servo on pin 9.
 */

#include <Servo.h>

// ----- Configuration -----
constexpr uint8_t SERVO_PIN = 9;
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr uint8_t START_BYTE = 0x02;
constexpr uint8_t END_BYTE = 0x03;
constexpr size_t SERIAL_BUFFER_SIZE = 32;

// ----- Globals -----
Servo headServo;
bool receiving_message = false;
int serial_index = 0;
int expected_length = -1;
byte serial_buffer[SERIAL_BUFFER_SIZE];

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.flush();
  headServo.attach(SERVO_PIN);
  headServo.write(90);
  Serial.println("head_pan.ino ready (servo on pin 9).");
}

void loop() {
  processSerialBuffer();
}

void processMessage(byte *data, int length) {
  char type = data[0];
  switch (type) {
    case 'S': {
      int angle = (int8_t)data[1];
      angle = constrain(angle, 0, 180);
      headServo.write(angle);
      break;
    }
    default:
      Serial.print("Unknown command: ");
      Serial.println(type);
      break;
  }
}

void processSerialBuffer() {
  while (Serial.available() > 0) {
    byte b = Serial.read();
    if (!receiving_message) {
      if (b == START_BYTE) {
        receiving_message = true;
        serial_index = 0;
        expected_length = -1;
      }
    } else {
      if (expected_length == -1) {
        expected_length = b;
        if (expected_length <= 0 || expected_length > SERIAL_BUFFER_SIZE) {
          receiving_message = false;
        }
      } else {
        serial_buffer[serial_index++] = b;
        if (serial_index == expected_length + 1) {
          if (serial_buffer[serial_index - 1] == END_BYTE) {
            processMessage(serial_buffer, expected_length);
          }
          receiving_message = false;
        } else if (serial_index >= SERIAL_BUFFER_SIZE) {
          receiving_message = false;
        }
      }
    }
  }
}
