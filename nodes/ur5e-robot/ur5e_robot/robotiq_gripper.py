"""Minimal Robotiq 2F-85 client — talks the URCap socket protocol (no SDK).

The Robotiq UR driver (URCap) running on the UR controller exposes a line-based
socket (default port 63352): ``SET <VAR> <n>`` / ``GET <VAR>`` over positions,
speed and force in the 0..255 range (0 = open, 255 = closed). This is plain
`socket` code — no `ur_rtde`/native deps — so it imports anywhere; only an actual
connection touches hardware.
"""

from __future__ import annotations

import socket
import time

OPEN, CLOSED = 0, 255  # Robotiq position range


class RobotiqGripper:
    def __init__(self, host: str, port: int = 63352, timeout: float = 2.0):
        self._sock = socket.create_connection((host, port), timeout=timeout)

    def _set(self, var: str, value: int) -> None:
        self._sock.sendall(f"SET {var} {int(value)}\n".encode())
        self._sock.recv(1024)  # "ack"

    def _get(self, var: str) -> int:
        self._sock.sendall(f"GET {var}\n".encode())
        reply = self._sock.recv(1024).decode().strip()  # e.g. "POS 128"
        return int(reply.split()[1])

    def activate(self) -> None:
        """Activate the gripper and block until it reports ready (STA == 3)."""
        self._set("ACT", 1)
        self._set("GTO", 1)
        while self._get("STA") != 3:
            time.sleep(0.1)

    def move(self, position: int, speed: int, force: int) -> None:
        """Command an absolute position (0 open .. 255 closed) at speed/force (0..255)."""
        self._set("SPE", max(0, min(255, speed)))
        self._set("FOR", max(0, min(255, force)))
        self._set("POS", max(OPEN, min(CLOSED, position)))

    def get_position(self) -> int:
        """Measured position (0 open .. 255 closed)."""
        return self._get("POS")

    def disconnect(self) -> None:
        self._sock.close()
