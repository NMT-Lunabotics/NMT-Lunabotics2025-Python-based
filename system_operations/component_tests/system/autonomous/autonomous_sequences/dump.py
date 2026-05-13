"""Autonomous routine: dumping sequence."""

from __future__ import annotations

SEQUENCE = [
    {"command": "A", "values": [-1, -1, -1, -1, 5, 0], "duration": 1},
    {"command": "M", "values": [16, 16], "duration": 1.5},
    {"command": "A", "values": [-1, -1, -1, -1, -5, 5], "duration": 1},
    {"command": "M", "values": [0, 0], "duration": 0.5},
    {"command": "A", "values": [-1, -1, -1, -1, 0, -5], "duration": 0.3},
    {"command": "M", "values": [-14, -14], "duration": 2.0},
    {"command": "M", "values": [0, 0], "duration": 2.0},
]


def get_sequence():
    return SEQUENCE
