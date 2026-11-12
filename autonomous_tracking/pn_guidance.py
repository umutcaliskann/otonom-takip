"""Proportional navigation style fixed-wing guidance."""

from __future__ import annotations

from math import degrees

from .ekf import LineOfSightState
from .geo import clamp
from .models import GuidanceCommand, TargetTelemetry, UavTelemetry


class ProportionalNavigationGuidance:
    def __init__(
        self,
        navigation_constant: float = 3.0,
        los_angle_gain: float = 70.0,
        los_rate_gain: float = 28.0,
        max_roll_deg: float = 35.0,
        max_pitch_deg: float = 15.0,
        max_yaw_rate_deg_s: float = 25.0,
    ) -> None:
        self.navigation_constant = navigation_constant
        self.los_angle_gain = los_angle_gain
        self.los_rate_gain = los_rate_gain
        self.max_roll_deg = max_roll_deg
        self.max_pitch_deg = max_pitch_deg
        self.max_yaw_rate_deg_s = max_yaw_rate_deg_s

    def command(
        self,
        ego: UavTelemetry,
        target: TargetTelemetry,
        los: LineOfSightState,
        airspeed_mps: float,
    ) -> GuidanceCommand:
        roll = self.los_angle_gain * los.azimuth_rad
        roll += self.navigation_constant * self.los_rate_gain * los.azimuth_rate_rad_s
        pitch = self.los_angle_gain * los.elevation_rad
        pitch += self.navigation_constant * self.los_rate_gain * los.elevation_rate_rad_s
        yaw_rate = self.navigation_constant * degrees(los.azimuth_rate_rad_s)
        return GuidanceCommand(
            mode="GUIDED",
            target_id=target.target_id,
            roll_deg=clamp(roll, -self.max_roll_deg, self.max_roll_deg),
            pitch_deg=clamp(pitch, -self.max_pitch_deg, self.max_pitch_deg),
            heading_deg=ego.heading_deg,
            yaw_rate_deg_s=clamp(yaw_rate, -self.max_yaw_rate_deg_s, self.max_yaw_rate_deg_s),
            airspeed_mps=airspeed_mps,
            source="visual_pn_guidance",
            timestamp_s=los.timestamp_s,
        )
