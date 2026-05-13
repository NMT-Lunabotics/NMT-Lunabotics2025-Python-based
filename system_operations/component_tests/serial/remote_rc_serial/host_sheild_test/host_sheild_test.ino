/*
 * Simple USB Host Shield Test
 * Confirms SPI communication and USB device detection.
 * For Arduino Mega 2560 + USB Host Shield
 */

 #include <SPI.h>
 #include <Usb.h>
 
 USB     Usb;      // core USB object
 
 void setup()
 {
   Serial.begin(115200);
   while (!Serial);  // wait for serial
   delay(2000);
 
   Serial.println(F("=== USB Host Shield Test ==="));
 
   if (Usb.Init() == -1) {
     Serial.println(F("USB Host Shield init failed!"));
     Serial.println(F("Check power, SPI jumpers (MEGA mode), and wiring."));
     while (1); // stop
   }
   Serial.println(F("USB Host Shield initialized."));
 }
 
 void loop()
 {
   Usb.Task();
 
   // print USB task state occasionally
   static uint8_t lastState = 0;
   uint8_t state = Usb.getUsbTaskState();
   if (state != lastState) {
     lastState = state;
     Serial.print(F("USB task state changed: "));
     Serial.println(state, HEX);
     if (state == USB_STATE_RUNNING) {
       Serial.println(F("✅ Device detected and running!"));
     }
   }
 
   delay(500);
 }
 