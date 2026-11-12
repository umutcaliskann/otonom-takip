"""Compensate delayed telemetry using speed and heading only."""

from __future__ import annotations

from dataclasses import replace
from typing import Iterable

from .geo import clamp, destination_point
from .models import TargetTelemetry, UavTelemetry


class DelayedTelemetryCompensator:
    """Forward-project delayed telemetry to the current processing timestamp.

    The project uses only the fields available in the competition feed:
    position, altitude, speed, heading, and timestamp. Vertical speed is not
    available, so altitude is held constant during compensation.
    """

    def __init__(self, max_prediction_dt_s: float = 2.0) -> None:
        self.max_prediction_dt_s = max_prediction_dt_s

    def compensate_ego(self, telemetry: UavTelemetry, current_time_s: float) -> UavTelemetry:
        dt = self._prediction_dt(telemetry.timestamp_s, current_time_s)
        latitude, longitude = destination_point(
            telemetry.latitude_deg,
            telemetry.longitude_deg,
            telemetry.speed_mps * dt,
            telemetry.heading_deg,
        )
        return replace(
            telemetry,
            latitude_deg=latitude,
            longitude_deg=longitude,
            timestamp_s=current_time_s,
        )

    def compensate_target(
        self,
        telemetry: TargetTelemetry,
        current_time_s: float,
    ) -> TargetTelemetry:
        dt = self._prediction_dt(telemetry.timestamp_s, current_time_s)
        latitude, longitude = destination_point(
            telemetry.latitude_deg,
            telemetry.longitude_deg,
            telemetry.speed_mps * dt,
            telemetry.heading_deg,
        )
        return replace(
            telemetry,
            latitude_deg=latitude,
            longitude_deg=longitude,
            timestamp_s=current_time_s,
        )

    def compensate_targets(
        self,
        targets: Iterable[TargetTelemetry],
        current_time_s: float,
    ) -> list[TargetTelemetry]:
        return [self.compensate_target(target, current_time_s) for target in targets]

    def _prediction_dt(self, telemetry_time_s: float, current_time_s: float) -> float:
        raw_dt = current_time_s - telemetry_time_s
        return clamp(raw_dt, 0.0, self.max_prediction_dt_s)
