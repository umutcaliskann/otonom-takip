"""Central safety limits and failsafe command generation."""

from __future__ import annotations

from dataclasses import replace

from .geo import clamp
from .models import GuidanceCommand, UavTelemetry


class SafetySupervisor:
    def __init__(
        self,
        max_roll_deg: float = 35.0,
        max_pitch_deg: float = 15.0,
        max_yaw_rate_deg_s: float = 25.0,
        min_airspeed_mps: float = 16.0,
        max_airspeed_mps: float = 30.0,
        safe_airspeed_mps: float = 22.0,
        close_distance_m: float = 45.0,
    ) -> None:
        self.max_roll_deg = max_roll_deg
        self.max_pitch_deg = max_pitch_deg
        self.max_yaw_rate_deg_s = max_yaw_rate_deg_s
        self.min_airspeed_mps = min_airspeed_mps
        self.max_airspeed_mps = max_airspeed_mps
        self.safe_airspeed_mps = safe_airspeed_mps
        self.close_distance_m = close_distance_m

    def apply_limits(
        self,
        command: GuidanceCommand,
        target_distance_m: float | None = None,
    ) -> GuidanceCommand:
        airspeed = clamp(command.airspeed_mps, self.min_airspeed_mps, self.max_airspeed_mps)
        if target_distance_m is not None and target_distance_m < self.close_distance_m:
            airspeed = self.min_airspeed_mps
        return replace(
            command,
            roll_deg=clamp(command.roll_deg, -self.max_roll_deg, self.max_roll_deg),
            pitch_deg=clamp(command.pitch_deg, -self.max_pitch_deg, self.max_pitch_deg),
            yaw_rate_deg_s=clamp(
                command.yaw_rate_deg_s,
                -self.max_yaw_rate_deg_s,
                self.max_yaw_rate_deg_s,
            ),
            airspeed_mps=airspeed,
        )

    def failsafe_command(
        self,
        ego: UavTelemetry,
        timestamp_s: float,
        reason: str,
    ) -> GuidanceCommand:
        return GuidanceCommand(
            mode="GUIDED",
            target_id=None,
            roll_deg=0.0,
            pitch_deg=0.0,
            heading_deg=ego.heading_deg,
            yaw_rate_deg_s=0.0,
            airspeed_mps=self.safe_airspeed_mps,
            source=f"failsafe:{reason}",
            timestamp_s=timestamp_s,
        )
