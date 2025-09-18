#define DACL1_PIN 2
#define DACR1_PIN 3

void setup() {
  Serial.begin(9600);
  pinMode(DACL1_PIN, OUTPUT); 
  pinMode(DACR1_PIN, OUTPUT); 

}

void loop() {
  analogWrite(DACL1_PIN, 0);
  analogWrite(DACR1_PIN, 50);
  delay(5000);
  analogWrite(DACL1_PIN, 0);
  analogWrite(DACR1_PIN, 0);
  delay(1000);
  analogWrite(DACL1_PIN, 50);
  analogWrite(DACR1_PIN, 0);
  delay(5000);
  analogWrite(DACL1_PIN, 0);
  analogWrite(DACR1_PIN, 0);
  delay(1000);
}
            