"""Camera calibration helpers for fixed nose-mounted cameras."""

from __future__ import annotations

from typing import Any

from .models import CameraIntrinsics


def load_camera_intrinsics(data: dict[str, Any]) -> CameraIntrinsics:
    width = int(data.get("image_width_px", 1280))
    height = int(data.get("image_height_px", 720))
    if width <= 0 or height <= 0:
        raise ValueError("Camera image dimensions must be positive.")

    if "focal_length_x_px" in data or "focal_length_y_px" in data:
        focal_x = float(data["focal_length_x_px"])
        focal_y = float(data.get("focal_length_y_px", focal_x))
        if focal_x <= 0.0 or focal_y <= 0.0:
            raise ValueError("Camera focal lengths must be positive.")
        return CameraIntrinsics(width, height, focal_x, focal_y)

    horizontal_fov = float(data.get("horizontal_fov_deg", 70.0))
    vertical_fov = data.get("vertical_fov_deg", 43.0)
    if horizontal_fov <= 0.0 or horizontal_fov >= 180.0:
        raise ValueError("horizontal_fov_deg must be in (0, 180).")
    if vertical_fov is not None:
        vertical_fov = float(vertical_fov)
        if vertical_fov <= 0.0 or vertical_fov >= 180.0:
            raise ValueError("vertical_fov_deg must be in (0, 180).")
    return CameraIntrinsics.from_fov(width, height, horizontal_fov, vertical_fov)
