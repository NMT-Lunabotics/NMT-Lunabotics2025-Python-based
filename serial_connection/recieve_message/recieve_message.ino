#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX
const int ledPin = 13;

void setup() {
  mySerial.begin(115200);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  if (mySerial.available()) {
    char c = mySerial.read();
    if (c == '1') digitalWrite(ledPin, HIGH);
    else if (c == '0') digitalWrite(ledPin, LOW);
  }
}
