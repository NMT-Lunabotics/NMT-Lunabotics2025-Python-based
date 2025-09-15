void setup() {
  Serial.begin(9600); // baud rate of (pi+arduino) 
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    Serial.print("Arduino got: ");
    Serial.println(msg);
  }
}

