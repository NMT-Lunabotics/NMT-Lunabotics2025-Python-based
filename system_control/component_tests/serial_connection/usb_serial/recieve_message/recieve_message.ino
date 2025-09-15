const int ledPin = 8; 

void setup() {
  Serial.begin(9600);     
  pinMode(ledPin, OUTPUT); 
}

void loop() {
  digitalWrite(ledPin, HIGH);
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();  // remove whitespace or newline characters

    // Power pin based on message
    if (msg == "1") {
      digitalWrite(ledPin, HIGH);  // Power pin
    } else {
      digitalWrite(ledPin, LOW);   // Unpower pin
    }
  }
}
