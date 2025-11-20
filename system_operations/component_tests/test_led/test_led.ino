const int ledPinOne = 3;
const int ledPinTwo = 5;
const int ledPinThree = 6;

int ledPinOneStore = 128;
int ledPinTwoStore = 128;
int ledPinThreeStore = 128;

void setup(){
    pinMode(ledPinOne, OUTPUT);
    pinMode(ledPinTwo, OUTPUT);
    pinMode(ledPinThree, OUTPUT);
}

void loop() {
    analogWrite(ledPinOne, ledPinOneStore);
    analogWrite(ledPinTwo, ledPinTwoStore);
    analogWrite(ledPinThree, ledPinThreeStore);
}



case 'D':
{    
  ledPinOneStore=(int8_t)data[1]; 
  ledPinTwoStore=(int8_t)data[2];  
  ledPinThreeStore=(int8_t)data[3]; 
  const char* str = "RGB colors set...";
  sendSerialFeedback('F', (uint8_t*)str, strlen(str));
  sendSerialFeedback('F', (uint8_t*){127}, 1);
  break;
}