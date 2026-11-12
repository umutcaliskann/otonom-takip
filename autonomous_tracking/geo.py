"""Geographic helpers for short-range UAV guidance."""

from __future__ import annotations

from math import asin, atan2, cos, degrees, hypot, radians, sin, sqrt

from .models import TargetTelemetry, UavTelemetry

EARTH_RADIUS_M = 6_371_000.0


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_degrees(angle_deg: float) -> float:
    """Wrap an angle to [-180, 180)."""

    return (angle_deg + 180.0) % 360.0 - 180.0


def horizontal_distance_m(a: UavTelemetry, b: TargetTelemetry) -> float:
    lat1 = radians(a.latitude_deg)
    lat2 = radians(b.latitude_deg)
    dlat = lat2 - lat1
    dlon = radians(b.longitude_deg - a.longitude_deg)
    hav = sin(dlat / 2.0) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * atan2(sqrt(hav), sqrt(1.0 - hav))


def distance_3d_m(a: UavTelemetry, b: TargetTelemetry) -> float:
    horizontal = horizontal_distance_m(a, b)
    vertical = b.altitude_m - a.altitude_m
    return hypot(horizontal, vertical)


def bearing_deg(a: UavTelemetry, b: TargetTelemetry) -> float:
    lat1 = radians(a.latitude_deg)
    lat2 = radians(b.latitude_deg)
    dlon = radians(b.longitude_deg - a.longitude_deg)
    y = sin(dlon) * cos(lat2)
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    return (degrees(atan2(y, x)) + 360.0) % 360.0


def destination_point(
    latitude_deg: float,
    longitude_deg: float,
    distance_m: float,
    heading_deg: float,
) -> tuple[float, float]:
    """Project a point forward using a spherical-earth short-range model."""

    angular_distance = distance_m / EARTH_RADIUS_M
    bearing = radians(heading_deg)
    lat1 = radians(latitude_deg)
    lon1 = radians(longitude_deg)
    lat2 = asin(
        sin(lat1) * cos(angular_distance)
        + cos(lat1) * sin(angular_distance) * cos(bearing)
    )
    lon2 = lon1 + atan2(
        sin(bearing) * sin(angular_distance) * cos(lat1),
        cos(angular_distance) - sin(lat1) * sin(lat2),
    )
    return degrees(lat2), (degrees(lon2) + 540.0) % 360.0 - 180.0
