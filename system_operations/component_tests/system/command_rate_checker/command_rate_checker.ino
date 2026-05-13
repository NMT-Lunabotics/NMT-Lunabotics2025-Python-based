#include <Arduino.h>

#define MY_SERIAL_BUFFER_SIZE 128
byte serial_buffer[MY_SERIAL_BUFFER_SIZE];
bool receiving_message = false;
int serial_index = 0;
int expected_length = -1;

unsigned long last_report_time = 0;

int count_A = 0;
int count_M = 0;

int count_A_zero = 0;
int count_A_nonzero = 0;

int count_M_zero = 0;
int count_M_nonzero = 0;

void processMessage(byte *data, int length) {
  char type = data[0];

  if (type == 'A') {
    count_A++;
    int8_t aL_speed = -(int8_t)data[5];
    int8_t aB_speed = -(int8_t)data[6];
    if (aL_speed == 0 && aB_speed == 0) {
      count_A_zero++;
    } else {
      count_A_nonzero++;
    }
  }

  if (type == 'M') {
    count_M++;
    int8_t mR_speed = (int8_t)data[1];
    int8_t mL_speed = (int8_t)data[2];
    if (mR_speed == 0 && mL_speed == 0) {
      count_M_zero++;
    } else {
      count_M_nonzero++;
    }
  }
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

  unsigned long now = millis();
  if (now - last_report_time >= 1000) {
    Serial.print("A: ");
    Serial.print(count_A);
    Serial.print(" Zero: ");
    Serial.print(count_A_zero);
    Serial.print(" Not-zero: ");
    Serial.print(count_A_nonzero);

    Serial.print(" | M: ");
    Serial.print(count_M);
    Serial.print(" Zero: ");
    Serial.print(count_M_zero);
    Serial.print(" Not-zero: ");
    Serial.println(count_M_nonzero);

    count_A = 0;
    count_M = 0;
    count_A_zero = 0;
    count_A_nonzero = 0;
    count_M_zero = 0;
    count_M_nonzero = 0;

    last_report_time = now;
  }
}