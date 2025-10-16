import subprocess
import time
import serial
import serial.tools.list_ports

# arduinoConsole class/toolset which makes it easy to compile and upload sketches to your arduino without the regular arduino IDE software.
# calling compile_and_upload() will compile and upload the sketch to the connected arduino, using your spcfifced skeach, board, baudrate and port.

class arduinoConsole:
    def __init__(self, sketch_path: str, board: str="arduino:avr:mega", baudrate: int=115200, port:int=None, errors:bool=False) -> None:
        """Default function variables"""
        self.sketch_path = sketch_path
        self.baudrate = baudrate
        self.ser = None
        self.errors = errors
        if not board or not port:
            self.board, self.port = self.detect_board()
        else:
            self.board = board
            self.port = port

    def find_arduino(self)->str:
        """Search currently used ports for Arduino"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Arduino" in port.description or port.vid is not None:
                return port.device
        raise RuntimeError("Arduino not found")

    def compile_and_upload(self)->None:
        """Compile and upload the sketch to connected arduino"""
        verbose = "--verbose" if self.errors else ""
        subprocess.run(f'arduino-cli compile --fqbn {self.board} {verbose} "{self.sketch_path}"', shell=True, check=True)
        time.sleep(2)
        subprocess.run(f'arduino-cli upload -p {self.port} --fqbn {self.board} {verbose} "{self.sketch_path}"', shell=True, check=True)

    def detect_board(self):
        for port in serial.tools.list_ports.comports():
            desc = port.description.lower()
            if "uno r4" in desc or "renesas" in desc:
                return "arduino:renesas_uno:unor4wifi", port.device
            elif "uno" in desc or "atmega" in desc:
                return "arduino:avr:uno", port.device
            elif "mega" in desc:
                return "arduino:avr:mega", port.device
        raise RuntimeError("Arduino board not found")