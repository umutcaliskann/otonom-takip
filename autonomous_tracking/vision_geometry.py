"""Image-plane to line-of-sight geometry."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2

from .models import CameraIntrinsics, VisualMeasurement


@dataclass(frozen=True)
class PixelError:
    x_px: float
    y_px: float


@dataclass(frozen=True)
class LineOfSightMeasurement:
    azimuth_rad: float
    elevation_rad: float
    timestamp_s: float


def pixel_error(
    measurement: VisualMeasurement,
    camera: CameraIntrinsics,
) -> PixelError:
    image_center_x = camera.image_width_px / 2.0
    image_center_y = camera.image_height_px / 2.0
    return PixelError(
        x_px=measurement.bbox_center_x_px - image_center_x,
        y_px=image_center_y - measurement.bbox_center_y_px,
    )


def los_from_visual_measurement(
    measurement: VisualMeasurement,
    camera: CameraIntrinsics,
) -> LineOfSightMeasurement:
    error = pixel_error(measurement, camera)
    return LineOfSightMeasurement(
        azimuth_rad=atan2(error.x_px, camera.focal_length_x_px),
        elevation_rad=atan2(error.y_px, camera.focal_length_y_px),
        timestamp_s=measurement.timestamp_s,
    )
