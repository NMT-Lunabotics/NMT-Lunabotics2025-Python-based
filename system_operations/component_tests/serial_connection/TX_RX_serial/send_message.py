import serial
import time

# Check of other program or devices are using the serial port.
# Requires that the GPIO are change on the PI, add this to file and (disable login shell over serial) and (enable serial hardware) in settings sudo raspi-config
#sudo nano /boot/config.txt
#enable_uart=1
#dtoverlay=disable-bt

ser = serial.Serial('/dev/serial0', 9600, timeout=1)
time.sleep(2)

value = 0
while True:
    ser.write(f"{value}\n".encode('utf-8'))
    value = 1 - value  # toggles between 0 and 1
    time.sleep(1)