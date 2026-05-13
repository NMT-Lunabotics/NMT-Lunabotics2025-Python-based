"""Autonomous routine: lateral traverse."""

from __future__ import annotations

SEQUENCE = [
    {"command": "A", "values": [-1, -1, -1, -1, 0, 0], "duration": 0.2},
    {"command": "M", "values": [8, -8], "duration": 1.5},
    {"command": "M", "values": [-8, 8], "duration": 1.5},
    {"command": "M", "values": [5, 5], "duration": 3.0},
    {"command": "M", "values": [0, 0], "duration": 1.0},
]


def get_sequence():
    return SEQUENCE
