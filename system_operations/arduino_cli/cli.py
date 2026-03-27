import os
import subprocess
import time
import serial
import serial.tools.list_ports
import re
import shutil
from pathlib import Path

# arduinoConsole class/toolset which makes it easy to compile and upload sketches to your arduino without the regular arduino IDE software.
# calling compile_and_upload() will compile and upload the sketch to the connected arduino, using your spcfifced skeach, board, baudrate and port.

class arduinoConsole:
    def __init__(self, sketch_path: str, board: str=None, baudrate: int=115200, port:int=None, errors:bool=False) -> None:
        """Default function variables"""
        self.sketch_path = sketch_path
        self.baudrate = baudrate
        self.ser = None
        self.errors = errors
        self.cli_path = self.find_arduino_cli()
        if board == None:
            try:
                self.board = self.detect_board()
            except RuntimeError as e:
                print(f"Arduino board not detcted, using arduino:avr:mega...")
                self.board="arduino:avr:mega"
        else:
            self.board = board
        if port == None:
            self.port = self.find_arduino()
        else: 
            self.port = port

    def find_arduino(self)->str:
        """Search currently used ports for Arduino"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Arduino" in port.description or port.vid is not None:
                return port.device

    def find_arduino_cli(self) -> str:
        """Locate arduino-cli from PATH, env override, or common Windows install folders."""
        env_path = os.environ.get("ARDUINO_CLI_PATH")
        if env_path and Path(env_path).is_file():
            return str(Path(env_path))

        cli_name = "arduino-cli.exe" if os.name == "nt" else "arduino-cli"
        path_cli = shutil.which(cli_name) or shutil.which("arduino-cli")
        if path_cli:
            return path_cli

        home = Path.home()
        candidates = [
            home / "Downloads" / "arduino-cli_1.4.1_Windows_64bit" / "arduino-cli.exe",
            home / "Downloads" / "arduino-cli.exe",
            home / "AppData" / "Local" / "Arduino15" / "arduino-cli.exe",
            home / "AppData" / "Local" / "Programs" / "Arduino CLI" / "arduino-cli.exe",
            Path("C:/Program Files/Arduino CLI/arduino-cli.exe"),
            Path("C:/Program Files/Arduino/arduino-cli.exe"),
        ]
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)

        raise FileNotFoundError(
            "arduino-cli not found. Install it, add it to PATH, or set ARDUINO_CLI_PATH."
        )

    def compile_and_upload(self)->None:
        """Compile and upload the sketch to connected arduino"""
        verbose = ["--verbose"] if self.errors else []
        if self.board == "arduino:renesas_uno:unor4wifi": self.switch_to_nuc_architecture()
        try: 
            subprocess.run(
                [self.cli_path, "compile", "--fqbn", self.board, *verbose, str(self.sketch_path)],
                check=True,
            )
            time.sleep(2)
            subprocess.run(
                [self.cli_path, "upload", "-p", str(self.port), "--fqbn", self.board, *verbose, str(self.sketch_path)],
                check=True,
            )
        except FileNotFoundError as e:
            print(f"Error: {e}")
            exit(1)
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
            exit(1)

    def detect_board(self):
        for port in serial.tools.list_ports.comports():
            if port.vid == 0x2341 and port.pid == 0x1002:
                return "arduino:renesas_uno:unor4wifi"
            elif port.vid == 0x2341 and port.pid == 0x0043:
                return "arduino:avr:uno"
            elif port.vid == 0x2341 and port.pid == 0x0010: 
                return "arduino:avr:mega"
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
