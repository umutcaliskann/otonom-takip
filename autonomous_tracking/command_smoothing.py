"""Command rate limiting for smoother fixed-wing setpoints."""

from __future__ import annotations

from dataclasses import replace

from .geo import clamp
from .models import GuidanceCommand


class CommandSmoother:
    def __init__(
        self,
        max_delta_roll_deg: float = 8.0,
        max_delta_pitch_deg: float = 5.0,
        max_delta_airspeed_mps: float = 3.0,
    ) -> None:
        self.max_delta_roll_deg = max_delta_roll_deg
        self.max_delta_pitch_deg = max_delta_pitch_deg
        self.max_delta_airspeed_mps = max_delta_airspeed_mps
        self._previous: GuidanceCommand | None = None

    def reset(self) -> None:
        self._previous = None

    def smooth(self, command: GuidanceCommand) -> GuidanceCommand:
        if self._previous is None:
            self._previous = command
            return command
        smoothed = replace(
            command,
            roll_deg=self._limit_delta(
                command.roll_deg,
                self._previous.roll_deg,
                self.max_delta_roll_deg,
            ),
            pitch_deg=self._limit_delta(
                command.pitch_deg,
                self._previous.pitch_deg,
                self.max_delta_pitch_deg,
            ),
            airspeed_mps=self._limit_delta(
                command.airspeed_mps,
                self._previous.airspeed_mps,
                self.max_delta_airspeed_mps,
            ),
        )
        self._previous = smoothed
        return smoothed

    def _limit_delta(self, value: float, previous: float, max_delta: float) -> float:
        return previous + clamp(value - previous, -max_delta, max_delta)
