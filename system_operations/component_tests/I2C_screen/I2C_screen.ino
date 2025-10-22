#include <Wire.h>

class LCD2004 {
private:
    uint8_t addr;
    uint8_t cols, rows;
    uint8_t backlight = 0x08; // Backlight on
    void expanderWrite(uint8_t data) {
        Wire.beginTransmission(addr);
        Wire.write(data | backlight);
        Wire.endTransmission();
    }
    void pulseEnable(uint8_t data) {
        expanderWrite(data | 0x04); // EN
        delayMicroseconds(1);
        expanderWrite(data & ~0x04);
        delayMicroseconds(50);
    }
    void write4bits(uint8_t value) {
        expanderWrite(value);
        pulseEnable(value);
    }
    void send(uint8_t value, uint8_t mode) {
        write4bits((value & 0xF0) | mode);
        write4bits(((value << 4) & 0xF0) | mode);
    }
    void command(uint8_t value) { send(value, 0x00); }
    void writeChar(char c) { send(c, 0x01); } // RS=1

public:
    LCD2004(uint8_t address = 0x27, uint8_t cols_ = 20, uint8_t rows_ = 4): addr(address), cols(cols_), rows(rows_) {}
    void begin() {
        Wire.begin();
        delay(50);
        write4bits(0x30); delayMicroseconds(4500);
        write4bits(0x30); delayMicroseconds(4500);
        write4bits(0x30); delayMicroseconds(150);
        write4bits(0x20);
        command(0x28); // 4-bit, 2-line, 5x8
        command(0x08); // display off
        command(0x01); // clear
        delay(2);
        command(0x06); // entry mode
        command(0x0C); // display on, cursor off
    }
    void clear() {
        command(0x01);
        delay(2);
    }
    void setCursor(uint8_t col, uint8_t row) {
        static const uint8_t rowOffsets[4] = {0x00, 0x40, 0x14, 0x54};
        command(0x80 | (col + rowOffsets[row]));
    }
    void backlightOn(bool on = true) {
        backlight = on ? 0x08 : 0x00;
        expanderWrite(0);
    }
void screenPrint(const char* text, const char* staticPrefix[] = nullptr, uint16_t pageDelayMs = 3000) {
    size_t startIndex = 0;

    while (text[startIndex] != '\0') {
        clear();
        uint8_t row = 0;

        while (row < rows) {
            setCursor(0, row);

            // Print static prefix for this row if exists
            size_t prefixLen = 0;
            if (staticPrefix && staticPrefix[row]) {
                prefixLen = strlen(staticPrefix[row]);
                for (size_t i = 0; i < prefixLen && i < cols; i++)
                    writeChar(staticPrefix[row][i]);
            }

            size_t available = (prefixLen < cols) ? (cols - prefixLen) : 0;

            // Print dynamic text until newline or end of row
            size_t i;
            for (i = 0; i < available && text[startIndex] != '\0'; i++, startIndex++) {
                if (text[startIndex] == '\n') {
                    startIndex++;  // skip newline
                    break;         // move to next row
                }
                writeChar(text[startIndex]);
            }

            row++;
        }

        delay(pageDelayMs);
    }
}


};

LCD2004 lcd(0x27, 20, 4);

void setup() {
    lcd.begin();
}
void loop() {
      const char* staticPrefix[] = {
        "[F12] ", // row 0
        nullptr, // row 1
        nullptr,  // row 2 empty
        nullptr   // row 3 empty
    };
    const char* longMsg = "The id at the top left is planned to be an error indacator and this is an example of an long message which spans over multiple screens.\nThe end";
    lcd.screenPrint(longMsg, staticPrefix, 5000);
}



