"""Autonomous routine: lateral traverse."""

from __future__ import annotations

SEQUENCE = [
    {"command": "M", "values": [18, -18], "duration": 1.5},
    {"command": "M", "values": [-18, 18], "duration": 1.5},
    {"command": "M", "values": [22, 22], "duration": 3.0},
    {"command": "M", "values": [0, 0], "duration": 1.0},
]


def get_sequence():
    return SEQUENCE
