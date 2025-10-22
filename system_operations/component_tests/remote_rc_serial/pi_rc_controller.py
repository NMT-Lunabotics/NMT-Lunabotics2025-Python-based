import serial
import time

centers = [1640, 1615, 1624, 1622]
soft_zones = [50, 250, 50, 50]
max_joy_values = [1971, 1976, 2000, 2000]
min_joy_values = [1000, 1040, 1087, 1064]

ibus_port = '/dev/serial0'
arduino_port = '/dev/ttyACM0'
baudrate = 115200

ser_ibus = serial.Serial(ibus_port, baudrate, timeout=0.01)

ser_arduino = None
while ser_arduino is None:
    try:
        ser_arduino = serial.Serial(arduino_port, baudrate, timeout=0.01)
        print(f"Connected to Arduino on {arduino_port}")
    except serial.SerialException:
        print(f"Port {arduino_port} not available. Retrying...")
        time.sleep(1)

def map_value(val, in_min, in_max, out_min, out_max):
    if in_max - in_min == 0:
        return out_min
    return int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def parse_ibus_packet(packet):
    if len(packet) < 4:
        return None
    if packet[0] != 0x20 or packet[1] != 0x40:
        return None
    num_channels = (len(packet) - 4) // 2
    channels = []
    for i in range(num_channels):
        pos = 2 + i*2
        ch = packet[pos] | (packet[pos+1] << 8)
        channels.append(ch)
    return channels

def process_channels(channels):
    joystick = [0, 0, 0, 0]
    mapping = [3, 2, 0, 1]
    for i in range(4):
        val = channels[mapping[i]]
        if centers[i] - soft_zones[i] <= val <= centers[i] + soft_zones[i]:
            joystick[i] = 0
        elif val < centers[i] - soft_zones[i]:
            joystick[i] = map_value(val, min_joy_values[i], centers[i]-soft_zones[i], 255, 129)
            joystick[i] = min(joystick[i], 255)
        else:
            joystick[i] = map_value(val, centers[i]+soft_zones[i], max_joy_values[i], 0, 127)
            joystick[i] = min(joystick[i], 127)
    return joystick

def send_serial_command(command, data):
    start_byte = 0x02
    end_byte = 0x03
    buf = bytearray()
    buf.append(start_byte)
    buf.append(len(data)+1)
    buf.append(ord(command))
    buf.extend(data)
    buf.append(end_byte)
    ser_arduino.write(buf)

buffer = bytearray()
last_joystick = None
print("RC controller initialized.")

while True:
    data = ser_ibus.read(ser_ibus.in_waiting or 1)
    if data:
        buffer.extend(data)
        while len(buffer) >= 4:
            if buffer[0] == 0x20 and buffer[1] == 0x40:
                packet_len = 2 + ((len(buffer) - 4) // 2) * 2 + 2
                if len(buffer) >= packet_len:
                    packet = buffer[:packet_len]
                    buffer = buffer[packet_len:]
                    channels = parse_ibus_packet(packet)
                    if channels:
                        joystick = process_channels(channels)
                        if joystick != last_joystick:
                            send_serial_command('L', bytes([joystick[1], joystick[2], joystick[3]]))
                            last_joystick = joystick
            else:
                buffer.pop(0)
