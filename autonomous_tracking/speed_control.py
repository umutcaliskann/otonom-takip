"""Target-distance and bbox-size based speed control."""

from __future__ import annotations

from typing import Optional

from .geo import clamp
from .models import CameraIntrinsics, VisualMeasurement


class SpeedController:
    def __init__(
        self,
        min_airspeed_mps: float = 16.0,
        cruise_airspeed_mps: float = 22.0,
        max_airspeed_mps: float = 30.0,
        far_distance_m: float = 500.0,
        near_distance_m: float = 90.0,
        small_bbox_ratio: float = 0.004,
        large_bbox_ratio: float = 0.08,
    ) -> None:
        self.min_airspeed_mps = min_airspeed_mps
        self.cruise_airspeed_mps = cruise_airspeed_mps
        self.max_airspeed_mps = max_airspeed_mps
        self.far_distance_m = far_distance_m
        self.near_distance_m = near_distance_m
        self.small_bbox_ratio = small_bbox_ratio
        self.large_bbox_ratio = large_bbox_ratio

    def command_speed(
        self,
        distance_m: Optional[float],
        visual: Optional[VisualMeasurement],
        camera: Optional[CameraIntrinsics],
    ) -> float:
        score = 0.5
        if distance_m is not None:
            distance_score = (distance_m - self.near_distance_m) / (
                self.far_distance_m - self.near_distance_m
            )
            score = clamp(distance_score, 0.0, 1.0)
        if visual is not None and camera is not None:
            image_area = camera.image_width_px * camera.image_height_px
            bbox_ratio = visual.bbox_area_px / image_area if image_area > 0 else 0.0
            bbox_score = 1.0 - (bbox_ratio - self.small_bbox_ratio) / (
                self.large_bbox_ratio - self.small_bbox_ratio
            )
            score = (score + clamp(bbox_score, 0.0, 1.0)) / 2.0
        return self.min_airspeed_mps + score * (self.max_airspeed_mps - self.min_airspeed_mps)
