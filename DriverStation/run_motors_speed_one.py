#!/usr/bin/env python3
"""Fire-and-forget helper that spins both drive motors at speed 1."""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from system_operations.arduino_serial_commuication.arduino_serial_commuication import (
    serialCommands,
)


def main() -> None:
    link = serialCommands()
    try:
        while True:
            
            # Command 'M' expects [left_speed, right_speed]; 1 keeps motion minimal.
            link.send_command("A", [-1, -1, -1, -1, 0, -10])
            for i in range(500):
                speed = int(round(i / 1000 * 30))
                link.send_command("M", [speed, speed])

                time.sleep(0.01)
            for i in range(500):
                speed = int(round(i / 1000 * 30))
                link.send_command("M", [15-speed, 15-speed])

                time.sleep(0.01)
    finally:
        link.close_serial()


if __name__ == "__main__":
    main()
