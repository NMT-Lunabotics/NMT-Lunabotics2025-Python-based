#define DACL1_PIN 3
#define DACL2_PIN 5
#define DACR1_PIN 6
#define DACR2_PIN 9

void setup() {
  pinMode(DACL1_PIN, OUTPUT); 
  pinMode(DACR1_PIN, OUTPUT); 
  pinMode(DACL2_PIN, OUTPUT); 
  pinMode(DACR2_PIN, OUTPUT); 
  // put your setup code here, to run once:

}

void loop() {
  analogWrite(DACL1_PIN, 255);
   analogWrite(DACL2_PIN, 255);
  analogWrite(DACR1_PIN, 255);
  analogWrite(DACR2_PIN, 255);
  // put your main code here, to run repeatedly:

}
            