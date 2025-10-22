import subprocess
import time
import serial
import serial.tools.list_ports
import re
from pathlib import Path

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
        if self.board == "arduino:renesas_uno:unor4wifi": self.switch_to_nuc_architecture()
        subprocess.run(f'arduino-cli compile --fqbn {self.board} {verbose} "{self.sketch_path}"', shell=True, check=True)
        time.sleep(2)
        subprocess.run(f'arduino-cli upload -p {self.port} --fqbn {self.board} {verbose} "{self.sketch_path}"', shell=True, check=True)

    def detect_board(self):
        for port in serial.tools.list_ports.comports():
            if port.vid == 0x2341 and port.pid == 0x1002:
                return "arduino:renesas_uno:unor4wifi", port.device
            elif port.vid == 0x2341 and port.pid == 0x0043:
                return "arduino:avr:uno", port.device
            elif port.vid == 0x2341 and port.pid == 0x0010: 
                return "arduino:avr:mega", port.device
        raise RuntimeError("Arduino board not found")
    
    def switch_to_nuc_architecture(self):
        """Automatically switch to nuc robot settings if nuc robot is detected"""
        ino_file = Path(self.sketch_path)
        if not ino_file.exists() or ino_file.is_dir():
            raise FileNotFoundError(f"Expected .ino file, got {ino_file}")
        text = ino_file.read_text().splitlines()
        new_lines = []
        changed = False
        for line in text:
            if re.match(r'^\s*#define\s+MAIN_ROBOT\b', line):
                new_lines.append("#define MAIN_ROBOT 0")
                changed = True
            else:
                new_lines.append(line)
        if changed:
            ino_file.write_text("\n".join(new_lines))
            print(f"Switched to nuc robot settings architecture")
        else:
            print(f"Using main robot settings")
