import serial
import time

ser = serial.Serial("COM8", 9600, timeout=1)
time.sleep(2) 

# startByte, lengthOfBytes, Command i.e. M, leftMotor, rightMotor, endByte
data = bytes([0x02, 0x03, 0x4D, 0x7F, 0x7F, 0x03])

# startByte, lengthOfBytes, Command i.e. A, arm target(2 bytes), bucket target(2 bytes), arm speed, bucket speed, endByte
#data = bytes([0x02, 0x07, 0x41, 0x00, 0x00, 0x00, 0x64, 0x00, 0x0A, 0x03])



ser.write(data)
ser.close()
