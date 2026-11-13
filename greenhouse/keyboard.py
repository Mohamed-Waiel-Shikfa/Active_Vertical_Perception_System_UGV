# SPDX-License-Identifier: GPL-3.0-only
"""Decode real terminal arrow sequences without confusing their ESC with quit."""
from collections.abc import Container


class KeyDecoder:
    def __init__(self):
        self.buffer = ""
        self.escape_since = None

    def feed(self, text: str, now: float) -> list[str]:
        """Consume raw terminal input and return completed keys in order.

        Keys are "up", "down", "left", "right", "a", "q", "c", "stop" (space) and "quit" (Esc, Ctrl-C).
        `now` is wall-clock seconds on a monotonic clock; a lone ESC becomes "quit" after 0.06 s.
        """
        self.buffer += text
        keys = []
        arrows = {"A": "up", "B": "down", "C": "right", "D": "left"}
        while self.buffer:
            if self.buffer[0] != "\x1b":
                key, self.buffer = self.buffer[0].lower(), self.buffer[1:]
                self.escape_since = None
                if key in ("a", "q", "c", " ", "\x03"):
                    keys.append({" ": "stop", "\x03": "quit"}.get(key, key))
                continue
            if self.escape_since is None:
                self.escape_since = now
            if len(self.buffer) >= 2 and self.buffer[1] in ("[", "O"):
                end = next((i for i in range(2, len(self.buffer))
                            if "@" <= self.buffer[i] <= "~"), None)
                if end is not None:
                    if end == 2 and self.buffer[end] in arrows:
                        keys.append(arrows[self.buffer[end]])
                    self.buffer = self.buffer[end + 1:]
                    self.escape_since = None
                elif now - self.escape_since >= 0.06:
                    self.buffer = ""
                    self.escape_since = None
                else:
                    break
            elif len(self.buffer) == 1 and now - self.escape_since >= 0.06:
                keys.append("quit")
                self.buffer = ""
                self.escape_since = None
            elif len(self.buffer) >= 2:
                keys.append("quit")
                self.buffer = ""
                self.escape_since = None
            else:
                break
        return keys


def velocities(pressed: Container[str], speed: float, turn: float) -> tuple[float, float, float]:
    """Map held keys to (forward m/s, yaw rad/s counterclockwise, lift +1 for A, -1 for Q, else 0).

    `speed` (m/s) and `turn` (rad/s) are the magnitudes; velocities are in the robot body frame.
    """
    return ((int("up" in pressed) - int("down" in pressed)) * speed,
            (int("left" in pressed) - int("right" in pressed)) * turn,
            float(int("a" in pressed) - int("q" in pressed)))
