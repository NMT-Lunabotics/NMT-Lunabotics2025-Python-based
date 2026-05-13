# executor.py
import time
from .autonomous_sequences.excavation import get_sequence as get_excavation_sequence
from .autonomous_sequences.dump import get_sequence as get_dump_sequence
from .autonomous_sequences.transverse import get_sequence as get_transverse_sequence
from .autonomous_sequences.showcase import get_sequence as get_showcase_sequence

AUTO_PROGRAMS = {
    "excavation": get_excavation_sequence,
    "dump": get_dump_sequence,
    "transverse": get_transverse_sequence,
    "showcase": get_showcase_sequence,
}

class AutonomousRunner:
    def __init__(self, serial):
        self.serial = serial
        self.sequence = []
        self.current_step = 0
        self.step_start_time = 0.0
        self.step_end_time = 0.0
        self.active = False

    def load_sequence(self, program_name: str):
        loader = AUTO_PROGRAMS.get(program_name)
        if loader:
            self.sequence = list(loader())
            self.current_step = 0
            self.active = True
            self.step_start_time = 0.0
            self.step_end_time = 0.0

    def update(self):
        if not self.active or self.current_step >= len(self.sequence):
            self.active = False
            return

        now = time.monotonic()
        step = self.sequence[self.current_step]

        if self.step_start_time == 0.0:
            self.step_start_time = now
            self.step_end_time = now + step.get("duration", 0.0)
            command = step.get("command")
            values = step.get("values", [])
            if command:
                self.serial.send_command(command, values)

        elif now >= self.step_end_time:
            self.current_step += 1
            self.step_start_time = 0.0
            self.step_end_time = 0.0

    def stop(self):
        self.active = False
        self.current_step = len(self.sequence)
