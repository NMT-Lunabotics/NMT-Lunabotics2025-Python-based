#sudo apt update
#sudo apt install python3-serial


import serial
import time

# adjust port if needed (check with ls /dev/tty*)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # wait for Arduino to reset

ser.write(b"Hello Arduino!\n")
print(ser.readline().decode('utf-8').strip())
