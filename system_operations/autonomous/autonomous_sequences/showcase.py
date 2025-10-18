"""Autonomous routine: traverse showcase."""

from __future__ import annotations

SEQUENCE = [
    {"command": "A", "values": [-1, -1, -1, -1, 0, 0], "duration": 0.2},
    {"command": "M", "values": [10, 10], "duration": 3},
    {"duration": 0.1},
    {"command": "M", "values": [-10, -10], "duration": 3},
    {"command": "M", "values": [8, -8], "duration": 1.5},
    {"command": "M", "values": [-8, 8], "duration": 1.5},
    {"command": "M", "values": [2, 2], "duration": 3.0},
    {"command": "M", "values": [0, 0], "duration": 1.0},
]


def get_sequence():
    return SEQUENCE
