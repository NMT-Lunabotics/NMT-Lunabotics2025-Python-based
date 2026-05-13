#!/usr/bin/env python3
import socket
import subprocess
import shlex

class NetworkingOperations:
    def __init__(self, port: int = 10001)->None:
        """Setup initial variables and UDP socket."""
        PORT = port                                                         # GUI command port
        self.address=("127.0.0.1",PORT)                                     # Localhost address                                          
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)           # Create UDP socket
        self.s.bind(("0.0.0.0", PORT))                                      # Bind to all interfaces
        self.s.setblocking(False)                                           # Set socket to non-blocking mode

    def run_command(self, command: str)->None:
        """Take a user defined CMD and execute it safely in the shell terminal."""
        try:
            args = shlex.split(command)                                     # Split command into arguments
            result = subprocess.run(args, capture_output=True, text=True)   # Execute command
            if result.stdout:                                               # Output command result
                print(result.stdout.strip())
            if result.stderr:                                               # Output command errors
                print(f"[Error] {result.stderr.strip()}")
        except Exception as e:                                              # Catch any execution errors
            print(f"[Error] Failed to execute command: {e}")

    def send_data(self, message: str, addr: tuple)->None:
        """Send data to a given address (IP, port) via UDP."""
        try:
            self.s.sendto(message.encode(), addr)
        except Exception as e:
            print(f"[Error] Failed to send data: {e}")

    def receive_data(self)->str:
        """Receive a single UDP packet and return message."""
        try:
            data, addr = self.s.recvfrom(2048)
            message = data.decode().strip() if data else None
            # Execute commands starting with "CMD:"
            if message and message.startswith("CMD:"):
                cmd_to_run = message[4:].strip()
                self.run_command(cmd_to_run)
            return message
        except Exception as e:
            print(f"[Error] Failed to receive packet: {e}")
            return None
