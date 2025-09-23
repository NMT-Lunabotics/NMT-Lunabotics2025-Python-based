import subprocess
import time
import serial
import serial.tools.list_ports

# arduinoConsole class/toolset which makes it easy to compile and upload sketches to your arduino without the regular arduino IDE software.
# calling compile_and_upload() will compile and upload the sketch to the connected arduino, using your spcfifced skeach, board, baudrate and port.

class arduinoConsole:
    def __init__(self, sketch_path, board="arduino:avr:mega", baudrate=115200, port=None, errors=False):
        """Default function variables"""
        self.sketch_path = sketch_path
        self.board = board
        self.baudrate = baudrate
        self.port = port or self.find_arduino()
        self.ser = None
        self.errors = errors

    def find_arduino(self):
        """Search currently used ports for Arduino"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Arduino" in port.description or port.vid is not None:
                return port.device
        raise RuntimeError("Arduino not found")

    def compile_and_upload(self):
        """Compile and upload the sketch to connected arduino"""
        verbose = "--verbose" if self.errors else ""
        subprocess.run(f'arduino-cli compile --fqbn {self.board} {verbose} "{self.sketch_path}"', shell=True, check=True)
        time.sleep(2)
        subprocess.run(f'arduino-cli upload -p {self.port} --fqbn {self.board} {verbose} "{self.sketch_path}"', shell=True, check=True)