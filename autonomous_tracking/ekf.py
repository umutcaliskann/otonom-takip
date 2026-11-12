"""Small EKF for LOS angle and LOS-rate estimation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .vision_geometry import LineOfSightMeasurement


@dataclass(frozen=True)
class LineOfSightState:
    azimuth_rad: float
    elevation_rad: float
    azimuth_rate_rad_s: float
    elevation_rate_rad_s: float
    timestamp_s: float


class LineOfSightEKF:
    """Constant-velocity Kalman filter for [az, el, az_rate, el_rate]."""

    def __init__(
        self,
        process_noise: float = 0.08,
        measurement_noise: float = 0.035,
        initial_variance: float = 1.0,
    ) -> None:
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.initial_variance = initial_variance
        self._x: Optional[np.ndarray] = None
        self._p = np.eye(4) * initial_variance
        self._timestamp_s: Optional[float] = None

    def reset(self) -> None:
        self._x = None
        self._p = np.eye(4) * self.initial_variance
        self._timestamp_s = None

    def update(
        self,
        measurement: LineOfSightMeasurement,
        telemetry_los: Optional[LineOfSightMeasurement] = None,
    ) -> LineOfSightState:
        if self._x is None:
            self._x = np.array(
                [measurement.azimuth_rad, measurement.elevation_rad, 0.0, 0.0],
                dtype=float,
            )
            self._timestamp_s = measurement.timestamp_s
            return self.state

        previous_timestamp = (
            measurement.timestamp_s if self._timestamp_s is None else self._timestamp_s
        )
        dt = max(1e-3, measurement.timestamp_s - previous_timestamp)
        self._predict(dt)
        self._correct(measurement, self.measurement_noise)
        if telemetry_los is not None:
            self._correct(telemetry_los, self.measurement_noise * 2.5)
        self._timestamp_s = measurement.timestamp_s
        return self.state

    @property
    def state(self) -> LineOfSightState:
        if self._x is None or self._timestamp_s is None:
            return LineOfSightState(0.0, 0.0, 0.0, 0.0, 0.0)
        return LineOfSightState(
            azimuth_rad=float(self._x[0]),
            elevation_rad=float(self._x[1]),
            azimuth_rate_rad_s=float(self._x[2]),
            elevation_rate_rad_s=float(self._x[3]),
            timestamp_s=self._timestamp_s,
        )

    def _predict(self, dt: float) -> None:
        assert self._x is not None
        f = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        q = np.eye(4) * self.process_noise * dt
        self._x = f @ self._x
        self._p = f @ self._p @ f.T + q

    def _correct(self, measurement: LineOfSightMeasurement, noise: float) -> None:
        assert self._x is not None
        h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=float)
        z = np.array([measurement.azimuth_rad, measurement.elevation_rad], dtype=float)
        r = np.eye(2) * noise
        innovation = z - h @ self._x
        s = h @ self._p @ h.T + r
        k = self._p @ h.T @ np.linalg.inv(s)
        self._x = self._x + k @ innovation
        self._p = (np.eye(4) - k @ h) @ self._p
