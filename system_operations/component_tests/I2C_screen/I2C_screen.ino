#include <Wire.h>

class LCD2004 {
private:
    struct LogMessage {
        char code;
        int id;
        int priority;
        const char* msg;
        unsigned long time;
    };

    #define MAX_LOGS 20
    LogMessage logBuffer[MAX_LOGS];
    bool showErrorId;
    bool showErrorIdBetweenScreens;
    int logCount = 0;
    uint8_t addr, cols, rows;
    uint8_t backlight = 0x08;

    void write(uint8_t data) {
        // Write I2C data to display at address and set backlight at same time
        Wire.beginTransmission(addr);
        Wire.write(data | backlight);
        Wire.endTransmission();
    }
    void pulseEnable(uint8_t data) {
        // Pulses lcd enable pin to tell it to start recviving commands
        write(data | 0x04);
        delayMicroseconds(1);
        write(data & ~0x04);
        delayMicroseconds(50);
    }
    void write4bits(uint8_t value) {
        // Enables lcd and latches the lcd to I2C data
        write(value);
        pulseEnable(value);
    }
    void send(uint8_t value, uint8_t mode) {
        write4bits((value & 0xF0) | mode);
        write4bits(((value << 4) & 0xF0) | mode);
    }
    void command(uint8_t value) { 
        // Sends command to lcd using 0x00
        send(value, 0x00); 
    }
    void writeChar(char c) { 
        // Writes character to lcd using 0x01
        send(c, 0x01); 
    }
    void clear() {
        // Clear display by sending a blank msg using command
        command(0x01);
        delay(2);
    }
    void setCursor(uint8_t col, uint8_t row) {
        // Set cursor position on screen
        static const uint8_t rowOffsets[4] = {0x00, 0x40, 0x14, 0x54};
        command(0x80 | (col + rowOffsets[row]));
    }

    void cleanExpiredMessages() {
        unsigned long now = millis();
        for (int i = 0; i < logCount;) {
            if (logBuffer[i].time != (unsigned long)-1 && now > logBuffer[i].time) {
                for (int j = i; j < logCount - 1; j++) logBuffer[j] = logBuffer[j + 1];
                logCount--;
            } else i++;
        }
    }

    int getMessagesByCode(char code, LogMessage* out[], int maxCount) {
        cleanExpiredMessages();
        int count = 0;
        for (int i = 0; i < logCount && count < maxCount; i++) {
            if (logBuffer[i].code == code) out[count++] = &logBuffer[i];
        }
        if (count > 1) {
            qsort(out, count, sizeof(LogMessage*), [](const void* a, const void* b) -> int {
                LogMessage* m1 = *(LogMessage**)a;
                LogMessage* m2 = *(LogMessage**)b;
                return m1->priority - m2->priority;
            });
        }
        return count;
    }



public:
    // Lcd class with it's address, and size
    LCD2004(uint8_t address = 0x27, uint8_t cols_ = 20, uint8_t rows_ = 4) : addr(address), cols(cols_), rows(rows_) {}
    void begin() {
        // Start wire I2C connection
        Wire.begin();
        delay(50);
        // Start talking to screen while it is initalizing, then swich to 4-bit mode(uses less pins) 
        write4bits(0x30); delayMicroseconds(4500);
        write4bits(0x30); delayMicroseconds(4500);
        write4bits(0x30); delayMicroseconds(150);
        write4bits(0x20);
        // Set default display settings, turn of display(stops flickers), clear display, set default writing mode, turn on display with no cursor
        command(0x28);
        command(0x08);
        command(0x01);
        delay(2);
        command(0x06);
        command(0x0C);
    }
    void backlightOn(bool on = true) {
        // Turn lcd screen backlight on/off
        backlight = on ? 0x08 : 0x00;
        write(0);
    }

    void displayLogs(uint16_t pageDelayMs = 3000) {
        // Every loop clear old messages and clear screen if we do not have messages to display
        cleanExpiredMessages();
        if (logCount == 0) { clear(); return; }
        //  Loop through predefined data types to display on cycle
        char codes[] = { 'F','E','C' };
        for (int c = 0; c < 3; c++) {
            LogMessage* messages[MAX_LOGS];
            int count = getMessagesByCode(codes[c], messages, MAX_LOGS);
            if (count == 0) continue;
    
            // Build a single text stream for all messages of this code
            String combinedText = "";
            for (int i = 0; i < count; i++) {
                combinedText += messages[i]->msg;
                combinedText += " "; // separator between messages
            }
    
            size_t textIndex = 0;
            while (textIndex < combinedText.length()) {
                clear();
                uint8_t row = 0;
    
                // Print header
                char header[8];
                bool singleMsg = (count == 1);
                if (singleMsg) snprintf(header, sizeof(header), "[%c%02d] ", messages[0]->code, messages[0]->id);
                else snprintf(header, sizeof(header), "[%c] ", messages[0]->code);
                uint8_t col = 0;
                setCursor(0, row);
                for (size_t h = 0; header[h] && col < cols; h++, col++) writeChar(header[h]);
    
                // Fill remaining space on screen
                while (row < rows && textIndex < combinedText.length()) {
                    setCursor(col, row);
                    while (col < cols && textIndex < combinedText.length()) {
                        char c = combinedText[textIndex++];
                        if (c == '\n') { col = cols; break; } // force newline
                        writeChar(c);
                        col++;
                    }
                    row++;
                    col = 0;
                }
    
                delay(pageDelayMs);
            }
        }
    
        // Clear if nothing left
        if (logCount == 0) clear();
    }
    

    void setMessageLog(char code, int id, int priority, const char* msg, long timeSeconds = -1, bool showErrorId = true, bool showErrorIdBetweenScreens = true) {
        unsigned long set_time = (timeSeconds > 0) ? timeSeconds * 1000 + millis() : (unsigned long)-1;
        LogMessage entry = { code, id, priority, msg, set_time, showErrorId, showErrorIdBetweenScreens };
        for (int i = 0; i < logCount; i++) {
            if (logBuffer[i].code == code && logBuffer[i].id == id) {
                logBuffer[i] = entry;
                return;
            }
        }
        if (logCount < MAX_LOGS) logBuffer[logCount++] = entry;
        else Serial.println("Log buffer full");
    }
    
};





LCD2004 lcd(0x27, 20, 4);
void setup() {
    lcd.begin();
    lcd.setMessageLog('F', 1, 0, "Fault test 1", 25);
    lcd.setMessageLog('F', 2, 0, "Fault test 2", 25);
    lcd.setMessageLog('F', 3, 0, "Fault test 3", 25);
    lcd.setMessageLog('F', 4, 0, "Fault test 4 this is a very long message this should now span over twop or three pages for testing", 25);
    lcd.setMessageLog('F', 5, 0, "Fault test 5", -1);
    lcd.setMessageLog('C', 1, 0, "Com", 25);
    lcd.setMessageLog('E', 1, 0, "Error", 25);
}
void loop() {
    lcd.displayLogs(2000);


    //const char* staticPrefix[] = {
    //    "[F12] ", // row 0
    //    nullptr, // row 1
    //    nullptr,  // row 2 empty
    //    nullptr   // row 3 empty
    //};
    //const char* longMsg = "The id at the top left is planned to be an error indacator and this is an example of an long message which spans over multiple screens.\nThe end";
    //lcd.screenPrint(longMsg, staticPrefix, 5000);
}



