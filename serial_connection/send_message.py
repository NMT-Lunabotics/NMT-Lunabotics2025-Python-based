import serial
import time
import serial.tools.list_ports

ser = serial.Serial('COM8', 115200, timeout=1)
time.sleep(2)

ser.write(b'Hello Arduino\n')
print(ser.readline().decode().strip())

while True:
    line = ser.readline().decode().strip()
    if line:
        print("Arduino says:", line)

        
ser.close()

#ports = serial.tools.list_ports.comports()
#for port in ports:
    #print(f"{port.device}: {port.description}")