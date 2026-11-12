"""Long-running synthetic scenarios for system-level behavior checks."""

from __future__ import annotations

from dataclasses import replace

from .models import TargetBBox, TargetTelemetry, TrackingStatus, UavTelemetry
from .pipeline import AutonomousTrackingPipeline


class SyntheticScenario:
    def __init__(
        self,
        pipeline: AutonomousTrackingPipeline,
        duration_s: float = 30.0,
        dt_s: float = 0.2,
    ) -> None:
        self.pipeline = pipeline
        self.duration_s = duration_s
        self.dt_s = dt_s

    def run(self) -> list[TrackingStatus]:
        statuses: list[TrackingStatus] = []
        ego = UavTelemetry(39.0, 32.0, 1_000.0, 22.0, 90.0, 0.0)
        base_target = TargetTelemetry("target", 39.001, 32.004, 1_030.0, 20.0, 260.0, 0.0)
        steps = int(self.duration_s / self.dt_s)
        for index in range(steps):
            now = round(index * self.dt_s, 3)
            delayed_time = max(0.0, now - 0.35)
            target = replace(base_target, timestamp_s=delayed_time)
            ego_sample = replace(ego, timestamp_s=delayed_time)
            bbox = self._bbox_for_time(now)
            statuses.append(
                self.pipeline.process(
                    ego_sample,
                    [target],
                    bbox,
                    current_time_s=now,
                )
            )
        return statuses

    def _bbox_for_time(self, timestamp_s: float) -> TargetBBox | None:
        if timestamp_s < 2.0:
            return None
        if 8.0 <= timestamp_s < 8.8:
            return None
        if 18.0 <= timestamp_s < 22.0:
            return None
        drift = min(120.0, timestamp_s * 3.0)
        noise = 8.0 if int(timestamp_s * 10) % 2 == 0 else -8.0
        return TargetBBox(
            x_px=600.0 + drift + noise,
            y_px=320.0 + noise / 2.0,
            width_px=90.0,
            height_px=50.0,
            timestamp_s=timestamp_s,
            confidence=0.9,
        )
