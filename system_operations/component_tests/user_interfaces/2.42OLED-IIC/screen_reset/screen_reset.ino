#include <Wire.h>

#define OLED_ADDR 0x3C // I2C address of your screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define PAGES (SCREEN_HEIGHT / 8) // 8 pixels per page

// Send a single command
void sendCommand(uint8_t cmd) {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x00); // Command mode
  Wire.write(cmd);
  Wire.endTransmission();
}

// Send a single data byte
void sendData(uint8_t data) {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x40); // Data mode
  Wire.write(data);
  Wire.endTransmission();
}

// Set page and column
void setCursor(uint8_t page, uint8_t column) {
  sendCommand(0xB0 + page);           // Page
  sendCommand(0x00 + (column & 0x0F)); // Low column
  sendCommand(0x10 + ((column >> 4) & 0x0F)); // High column
}

// Fill screen with a byte pattern
void fillScreen(uint8_t pattern) {
  for (uint8_t page = 0; page < PAGES; page++) {
    setCursor(page, 0);
    for (uint8_t col = 0; col < SCREEN_WIDTH; col++) {
      sendData(pattern);
    }
  }
}

void setup() {
  Wire.begin();
  delay(100);

  // Basic init sequence
  sendCommand(0xAE); // Display off
  sendCommand(0xD5); sendCommand(0x80);
  sendCommand(0xA8); sendCommand(0x3F);
  sendCommand(0xD3); sendCommand(0x00);
  sendCommand(0x40);
  sendCommand(0x8D); sendCommand(0x14);
  sendCommand(0x20); sendCommand(0x00);
  sendCommand(0xA1); sendCommand(0xC8);
  sendCommand(0xDA); sendCommand(0x12);
  sendCommand(0x81); sendCommand(0xCF);
  sendCommand(0xD9); sendCommand(0xF1);
  sendCommand(0xDB); sendCommand(0x40);
  sendCommand(0xA4); sendCommand(0xA6);
  sendCommand(0xAF); // Display on
}

void loop() {
  fillScreen(0xFF); // Turn all pixels on
  delay(500);
  fillScreen(0x00); // Turn all pixels off
  delay(500);
}