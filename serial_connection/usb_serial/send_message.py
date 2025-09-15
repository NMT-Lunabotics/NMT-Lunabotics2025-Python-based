import serial
import time

# Check of other program or devices are using the serial port.
# lsof /dev/ttyACM0
# sudo kill -9 <PID>

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

value = 0
while True:
    ser.write(f"{value}\n".encode('utf-8'))
    value = 1 - value  # toggles between 0 and 1
    time.sleep(1)