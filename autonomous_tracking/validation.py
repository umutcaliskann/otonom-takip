"""Input validation for telemetry and target bbox data."""

from __future__ import annotations

from dataclasses import dataclass

from .models import CameraIntrinsics, TargetBBox, TargetTelemetry, UavTelemetry


@dataclass(frozen=True)
class ValidationResult:
    is_valid: bool
    reason: str | None = None
    quality: float = 1.0


class InputValidator:
    def __init__(
        self,
        max_telemetry_age_s: float = 2.0,
        max_bbox_age_s: float = 0.5,
        min_bbox_area_ratio: float = 0.0001,
        max_bbox_area_ratio: float = 0.75,
    ) -> None:
        self.max_telemetry_age_s = max_telemetry_age_s
        self.max_bbox_age_s = max_bbox_age_s
        self.min_bbox_area_ratio = min_bbox_area_ratio
        self.max_bbox_area_ratio = max_bbox_area_ratio

    def validate_ego_telemetry(
        self,
        telemetry: UavTelemetry,
        current_time_s: float,
    ) -> ValidationResult:
        return self._validate_telemetry_age(telemetry.timestamp_s, current_time_s)

    def validate_target_telemetry(
        self,
        telemetry: TargetTelemetry,
        current_time_s: float,
    ) -> ValidationResult:
        return self._validate_telemetry_age(telemetry.timestamp_s, current_time_s)

    def validate_bbox(
        self,
        bbox: TargetBBox | None,
        camera: CameraIntrinsics,
        current_time_s: float,
    ) -> ValidationResult:
        if bbox is None:
            return ValidationResult(False, "missing_bbox", 0.0)
        if bbox.width_px <= 0.0 or bbox.height_px <= 0.0:
            return ValidationResult(False, "invalid_bbox_size", 0.0)
        if current_time_s - bbox.timestamp_s > self.max_bbox_age_s:
            return ValidationResult(False, "stale_bbox", 0.0)
        if bbox.x_px >= camera.image_width_px or bbox.y_px >= camera.image_height_px:
            return ValidationResult(False, "bbox_outside_image", 0.0)
        if bbox.x_px + bbox.width_px <= 0.0 or bbox.y_px + bbox.height_px <= 0.0:
            return ValidationResult(False, "bbox_outside_image", 0.0)

        image_area = camera.image_width_px * camera.image_height_px
        area_ratio = bbox.width_px * bbox.height_px / image_area
        if area_ratio < self.min_bbox_area_ratio:
            return ValidationResult(False, "bbox_too_small", area_ratio)
        if area_ratio > self.max_bbox_area_ratio:
            return ValidationResult(False, "bbox_too_large", area_ratio)
        age_quality = max(0.0, 1.0 - (current_time_s - bbox.timestamp_s) / self.max_bbox_age_s)
        area_quality = min(1.0, area_ratio / max(self.min_bbox_area_ratio * 10.0, 1e-9))
        quality = max(0.0, min(1.0, bbox.confidence * 0.6 + age_quality * 0.2 + area_quality * 0.2))
        return ValidationResult(True, None, quality)

    def _validate_telemetry_age(
        self,
        telemetry_time_s: float,
        current_time_s: float,
    ) -> ValidationResult:
        age_s = current_time_s - telemetry_time_s
        if age_s < 0.0:
            return ValidationResult(False, "future_telemetry", 0.0)
        if age_s > self.max_telemetry_age_s:
            return ValidationResult(False, "stale_telemetry", 0.0)
        quality = max(0.0, 1.0 - age_s / self.max_telemetry_age_s)
        return ValidationResult(True, None, quality)
