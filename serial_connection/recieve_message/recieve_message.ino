void setup() {
  Serial.begin(115200);
  //Serial.flush();
}

void loop() {
  Serial.println("Test");
  delay(1000);
    //if (Serial.available() > 0) {
    //    String text = Serial.readStringUntil('\n');
    //    Serial.println(text);
    //}
}
