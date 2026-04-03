#include <Arduino.h>

#define MY_SERIAL_BUFFER_SIZE 128
byte serial_buffer[MY_SERIAL_BUFFER_SIZE];
bool receiving_message = false;
int serial_index = 0;
int expected_length = -1;

void processMessage(byte *data, int length) {
  char type = data[0];
  Serial.print("Command: ");
  Serial.print(type);
  Serial.print(" Data: ");
  for (int i = 1; i < length; i++) {
    int8_t signed_val = (int8_t)data[i];
    Serial.print(signed_val);
    Serial.print(" ");
  }
  Serial.println();
}

void processSerialBuffer() {
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
        if (expected_length <= 0 || expected_length > MY_SERIAL_BUFFER_SIZE) {
          receiving_message = false;
        }
      } else {
        serial_buffer[serial_index++] = b;
        if (serial_index == expected_length + 1) {
          if (serial_buffer[serial_index-1] == 0x03) {
            processMessage(serial_buffer, expected_length);
          } else {
            Serial.println("End byte not found");
          }
          receiving_message = false;
        } else if (serial_index >= MY_SERIAL_BUFFER_SIZE) {
          receiving_message = false;
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Command monitor ready");
}

void loop() {
  processSerialBuffer();
}