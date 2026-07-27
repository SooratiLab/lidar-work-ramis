"""Rate-limited decision gate for the optional Unitree StopMove adapter."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ActuationDecision:
    should_send_stop: bool
    reason: str


class StopCommandGate:
    """Send once on a stop edge, then periodically while stop remains true."""

    def __init__(self, repeat_interval: float = 1.0):
        if repeat_interval <= 0:
            raise ValueError("repeat_interval must be positive")
        self.repeat_interval = repeat_interval
        self._last_stop_sent = None
        self._request_active = False

    def update(self, stop_requested: bool, timestamp: float) -> ActuationDecision:
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")

        if not stop_requested:
            self._request_active = False
            self._last_stop_sent = None
            return ActuationDecision(False, "clear")

        first_request = not self._request_active
        self._request_active = True
        due = (
            first_request
            or self._last_stop_sent is None
            or timestamp < self._last_stop_sent
            or timestamp - self._last_stop_sent >= self.repeat_interval
        )
        if due:
            self._last_stop_sent = timestamp
            return ActuationDecision(
                True, "new_stop_request" if first_request else "repeat_stop_request"
            )
        return ActuationDecision(False, "rate_limited")
