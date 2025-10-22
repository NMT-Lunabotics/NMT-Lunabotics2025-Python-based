"""Autonomous routine: excavation pattern."""

from __future__ import annotations

SEQUENCE = [
    {"command": "A", "values": [-1, -1, -1, -1, 0, 10], "duration": 0.2},
    {"command": "M", "values": [10, 10], "duration": 4.0},
    {"command": "A", "values": [-1, -1, -1, -1, 0, -10], "duration": 0.2},
    {"command": "M", "values": [-10, -10], "duration": 3.0},
    {"command": "M", "values": [0, 0], "duration": 0.1},
    {"duration": 1.0},
] 


def get_sequence():
    return SEQUENCE
