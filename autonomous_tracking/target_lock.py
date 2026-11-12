"""Target ID lock and hysteresis for stable target selection."""

from __future__ import annotations

from typing import Iterable

from .models import TargetTelemetry, UavTelemetry
from .target_selection import TargetSelectionResult, TargetSelector


class TargetLockManager:
    def __init__(
        self,
        switch_distance_ratio: float = 0.75,
        switch_confirmations: int = 2,
    ) -> None:
        self.switch_distance_ratio = switch_distance_ratio
        self.switch_confirmations = switch_confirmations
        self.locked_target_id: str | None = None
        self._candidate_id: str | None = None
        self._candidate_count = 0

    def reset(self) -> None:
        self.locked_target_id = None
        self._candidate_id = None
        self._candidate_count = 0

    def select(
        self,
        ego: UavTelemetry,
        targets: Iterable[TargetTelemetry],
        selector: TargetSelector,
    ) -> TargetSelectionResult | None:
        target_list = list(targets)
        best = selector.select(ego, target_list)
        if best is None:
            self.reset()
            return None
        locked = self._locked_result(ego, target_list, selector)
        if locked is None:
            self.locked_target_id = best.target.target_id
            return best
        if best.target.target_id == locked.target.target_id:
            self._candidate_id = None
            self._candidate_count = 0
            return locked
        should_switch = best.distance_3d_m < locked.distance_3d_m * self.switch_distance_ratio
        if not should_switch:
            self._candidate_id = None
            self._candidate_count = 0
            return locked
        if self._candidate_id == best.target.target_id:
            self._candidate_count += 1
        else:
            self._candidate_id = best.target.target_id
            self._candidate_count = 1
        if self._candidate_count >= self.switch_confirmations:
            self.locked_target_id = best.target.target_id
            self._candidate_id = None
            self._candidate_count = 0
            return best
        return locked

    def _locked_result(
        self,
        ego: UavTelemetry,
        targets: list[TargetTelemetry],
        selector: TargetSelector,
    ) -> TargetSelectionResult | None:
        if self.locked_target_id is None:
            return None
        locked_targets = [target for target in targets if target.target_id == self.locked_target_id]
        if not locked_targets:
            self.reset()
            return None
        return selector.select(ego, locked_targets)
