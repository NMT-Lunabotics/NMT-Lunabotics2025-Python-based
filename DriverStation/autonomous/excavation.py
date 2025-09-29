"""Autonomous routine: excavation pattern."""

from __future__ import annotations

SEQUENCE = [
    {"command": "A", "values": [-1, -1, -1, -1, -20, 10], "duration": 0.2},
    {"command": "M", "values": [24, 24], "duration": 4.0},
    {"command": "A", "values": [-1, -1, -1, -1, 0, -30], "duration": 0.2},
    {"command": "M", "values": [-18, -18], "duration": 3.0},
    {"duration": 1.0},
]


def get_sequence():
    return SEQUENCE
