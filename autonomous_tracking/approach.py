"""Telemetry-based first approach logic."""

from __future__ import annotations

from .geo import bearing_deg, clamp, wrap_degrees
from .models import GuidanceCommand, TargetTelemetry, UavTelemetry


class ApproachController:
    def __init__(
        self,
        max_roll_deg: float = 30.0,
        max_pitch_deg: float = 12.0,
        heading_to_roll_gain: float = 0.45,
        altitude_to_pitch_gain: float = 0.015,
        approach_airspeed_mps: float = 24.0,
    ) -> None:
        self.max_roll_deg = max_roll_deg
        self.max_pitch_deg = max_pitch_deg
        self.heading_to_roll_gain = heading_to_roll_gain
        self.altitude_to_pitch_gain = altitude_to_pitch_gain
        self.approach_airspeed_mps = approach_airspeed_mps

    def command(
        self,
        ego: UavTelemetry,
        target: TargetTelemetry,
        timestamp_s: float,
    ) -> GuidanceCommand:
        desired_heading = bearing_deg(ego, target)
        heading_error = wrap_degrees(desired_heading - ego.heading_deg)
        altitude_error = target.altitude_m - ego.altitude_m
        roll = clamp(
            self.heading_to_roll_gain * heading_error,
            -self.max_roll_deg,
            self.max_roll_deg,
        )
        pitch = clamp(
            self.altitude_to_pitch_gain * altitude_error,
            -self.max_pitch_deg,
            self.max_pitch_deg,
        )
        return GuidanceCommand(
            mode="GUIDED",
            target_id=target.target_id,
            roll_deg=roll,
            pitch_deg=pitch,
            heading_deg=desired_heading,
            yaw_rate_deg_s=0.0,
            airspeed_mps=self.approach_airspeed_mps,
            source="telemetry_approach",
            timestamp_s=timestamp_s,
        )
