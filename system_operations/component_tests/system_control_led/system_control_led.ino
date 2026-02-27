#include "helpers.hpp"

// Led pins
#define LED_R_PIN 9
#define LED_G_PIN 10
#define LED_B_PIN 11

OutPin ledR(LED_R_PIN);
OutPin ledG(LED_G_PIN);
OutPin ledB(LED_B_PIN);

// Used variables
unsigned long current_time=0;
bool receiving_message=false;
int serial_index=0;
int expected_length=-1;
const int MY_SERIAL_BUFFER_SIZE=128;
byte serial_buffer[MY_SERIAL_BUFFER_SIZE];

// Change controller message into led colors
void processMessage(byte *data, int length) {
  char type = data[0];
  /*if(type=='M'){
    int8_t left = data[1];
    int8_t right = data[2];
    int r = map(left, -30, 30, 0, 255);
    int g = map(right, -30, 30, 0, 255);
    ledR.write(r);
    ledG.write(g);
    ledB.write(255);
  }*/
  if(type=='A'){
    int8_t left = data[5];
    int8_t right = data[6];
    int r = map(left, -25, 25, 0, 255);
    int g = map(right, -25, 25, 0, 255);
    ledR.write(r);
    ledG.write(g);
    ledB.write(255);
  }
}

// Process serial message in normal formate
void processSerialBuffer() {
  while(Serial.available()>0){
    byte b=Serial.read();
    if(!receiving_message){
      if(b==0x02){
        receiving_message=true;
        serial_index=0;
        expected_length=-1;
      }
    }else{
      if(expected_length==-1){
        expected_length=b;
        if(expected_length<=0||expected_length>MY_SERIAL_BUFFER_SIZE) receiving_message=false;
      }else{
        serial_buffer[serial_index++]=b;
        if(serial_index==expected_length+1){
          if(serial_buffer[serial_index-1]==0x03) processMessage(serial_buffer, expected_length);
          receiving_message=false;
        }else if(serial_index>=MY_SERIAL_BUFFER_SIZE) receiving_message=false;
      }
    }
  }
}

// Setup and running
void setup() {
  Serial.begin(115200);
}
void loop() {
  current_time=millis();
  processSerialBuffer();
}
