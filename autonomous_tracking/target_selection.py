"""Target selection using competition-server telemetry."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

from .geo import distance_3d_m, horizontal_distance_m
from .models import TargetTelemetry, UavTelemetry


@dataclass(frozen=True)
class TargetSelectionResult:
    target: TargetTelemetry
    horizontal_distance_m: float
    distance_3d_m: float
    altitude_difference_m: float


class TargetSelector:
    def __init__(
        self,
        max_distance_m: Optional[float] = None,
        max_altitude_difference_m: Optional[float] = None,
    ) -> None:
        self.max_distance_m = max_distance_m
        self.max_altitude_difference_m = max_altitude_difference_m

    def select(
        self,
        ego: UavTelemetry,
        targets: Iterable[TargetTelemetry],
    ) -> Optional[TargetSelectionResult]:
        candidates: list[TargetSelectionResult] = []
        for target in targets:
            horizontal = horizontal_distance_m(ego, target)
            distance = distance_3d_m(ego, target)
            altitude_difference = target.altitude_m - ego.altitude_m
            if self.max_distance_m is not None and distance > self.max_distance_m:
                continue
            if (
                self.max_altitude_difference_m is not None
                and abs(altitude_difference) > self.max_altitude_difference_m
            ):
                continue
            candidates.append(
                TargetSelectionResult(
                    target=target,
                    horizontal_distance_m=horizontal,
                    distance_3d_m=distance,
                    altitude_difference_m=altitude_difference,
                )
            )
        if not candidates:
            return None
        return min(candidates, key=lambda candidate: candidate.distance_3d_m)
