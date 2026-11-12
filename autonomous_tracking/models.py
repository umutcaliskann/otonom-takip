"""Shared data models for the autonomous tracking pipeline.

Units:
    - latitude/longitude: degrees
    - altitude: meters above the same reference datum
    - speed: meters/second
    - heading/bank/pitch: degrees
    - LOS angles and rates: radians and radians/second
    - timestamps: seconds
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import radians, tan
from typing import Optional


@dataclass(frozen=True)
class UavTelemetry:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    speed_mps: float
    heading_deg: float
    timestamp_s: float


@dataclass(frozen=True)
class TargetTelemetry:
    target_id: str
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    speed_mps: float
    heading_deg: float
    timestamp_s: float


@dataclass(frozen=True)
class VisualMeasurement:
    bbox_center_x_px: float
    bbox_center_y_px: float
    bbox_width_px: float
    bbox_height_px: float
    confidence: float
    timestamp_s: float

    @property
    def bbox_area_px(self) -> float:
        return max(0.0, self.bbox_width_px) * max(0.0, self.bbox_height_px)


@dataclass(frozen=True)
class TargetBBox:
    """Target bbox supplied by the competition system.

    The bbox uses top-left pixel coordinates plus width/height. If the upstream
    source does not provide a confidence score, keep the default value of 1.0.
    """

    x_px: float
    y_px: float
    width_px: float
    height_px: float
    timestamp_s: float = 0.0
    confidence: float = 1.0

    def to_visual_measurement(self) -> VisualMeasurement:
        return VisualMeasurement(
            bbox_center_x_px=self.x_px + self.width_px / 2.0,
            bbox_center_y_px=self.y_px + self.height_px / 2.0,
            bbox_width_px=self.width_px,
            bbox_height_px=self.height_px,
            confidence=self.confidence,
            timestamp_s=self.timestamp_s,
        )


@dataclass(frozen=True)
class CameraIntrinsics:
    image_width_px: int
    image_height_px: int
    focal_length_x_px: float
    focal_length_y_px: float

    @classmethod
    def from_fov(
        cls,
        image_width_px: int,
        image_height_px: int,
        horizontal_fov_deg: float,
        vertical_fov_deg: Optional[float] = None,
    ) -> "CameraIntrinsics":
        """Build camera intrinsics from horizontal and optional vertical FOV."""

        horizontal_fov_rad = radians(horizontal_fov_deg)
        if vertical_fov_deg is None:
            aspect = image_height_px / image_width_px
            focal_x = image_width_px / (2.0 * tan(horizontal_fov_rad / 2.0))
            focal_y = focal_x
        else:
            vertical_fov_rad = radians(vertical_fov_deg)
            focal_x = image_width_px / (2.0 * tan(horizontal_fov_rad / 2.0))
            focal_y = image_height_px / (2.0 * tan(vertical_fov_rad / 2.0))
        return cls(
            image_width_px=image_width_px,
            image_height_px=image_height_px,
            focal_length_x_px=focal_x,
            focal_length_y_px=focal_y,
        )


@dataclass(frozen=True)
class GuidanceCommand:
    mode: str
    target_id: Optional[str]
    roll_deg: float
    pitch_deg: float
    heading_deg: float
    yaw_rate_deg_s: float
    airspeed_mps: float
    source: str
    timestamp_s: float


class TrackingState(str, Enum):
    APPROACH = "APPROACH"
    TRACK = "TRACK"
    LOST = "LOST"
    FAILSAFE = "FAILSAFE"


@dataclass(frozen=True)
class TrackingStatus:
    state: TrackingState
    target_id: Optional[str]
    command: GuidanceCommand
    reason: Optional[str]
    measurement_quality: float
    timestamp_s: float
