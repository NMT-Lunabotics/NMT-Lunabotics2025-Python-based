import time
import serial
import serial.tools.list_ports

class serialCommands:
    def __init__(self, port=None, baudrate=115200):
        """Default function variables"""
        self.baudrate = baudrate
        self.port = port or self.find_arduino()
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.startByte=2
        self.endbyte=3
        self._read_buffer = bytearray()
        time.sleep(2)

    def find_arduino(self):
        """Auto-detect Arduino port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Arduino" in port.description or port.vid is not None:
                return port.device
        raise RuntimeError("Arduino not found")

    def send_serial(self, message: str):
        """Send a string to the Arduino"""
        self.ser.write((message + "\n").encode())
        self.ser.flush()

    def close_serial(self):
        """Close the serial connection"""
        if self.ser:
            self.ser.close()
            self.ser = None

    def read_raw_serial(self):
        """Reads the raw audio byte stream from Arduino"""
        return self.ser.read(self.ser.in_waiting or 1)
    
    def read_serial(self):
        """Read all available lines from Arduino and return as a list"""
        lines = []
        while self.ser.in_waiting > 0:
            line = self.ser.readline()
            if line:
                lines.append(line.decode(errors='ignore').strip())
        return lines if lines else None
    
    def send_command(self, command: str, data: list):
        """Send required data string command to arduino over serial."""
        length = len(data)+1 # Length = command byte + data bytes
        msg = bytearray()
        msg.append(self.startByte)
        msg.append(length)
        msg.append(ord(command))
        for d in data:
            msg.append(d % 256)
        msg.append(self.endbyte)
        self.ser.write(msg)
        self.ser.flush()
        time.sleep(0.001)

def read_command_feedback(self):
    """Read feedback from command operations"""
    packets = []
    while self.ser.in_waiting > 0:
        b = self.ser.read(1)[0]
        self._read_buffer.append(b)

        # Wait for start byte
        if len(self._read_buffer) == 1 and self._read_buffer[0] != self.startByte:
            self._read_buffer.clear()
            continue

        # Get expected length
        if len(self._read_buffer) == 2:
            expected_length = self._read_buffer[1]
            continue

        # Check if full packet received
        expected_length = self._read_buffer[1]
        if len(self._read_buffer) >= expected_length + 3:  # start + length + command/data + end
            if self._read_buffer[expected_length + 2] == self.endbyte:
                command = chr(self._read_buffer[2])
                data = list(self._read_buffer[3:3 + expected_length - 1])
                packets.append({"command": command, "data": data})
                # Remove the processed packet from buffer
                self._read_buffer = self._read_buffer[expected_length + 3:]
            else:
                # Bad packet, discard first byte
                self._read_buffer = self._read_buffer[1:]
    return packets if packets else None
