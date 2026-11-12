"""Tracking state transitions for the autonomous pipeline."""

from __future__ import annotations

from .models import TrackingState


class TrackingStateMachine:
    def __init__(self, lost_timeout_s: float = 1.0, reacquire_timeout_s: float = 3.0) -> None:
        self.lost_timeout_s = lost_timeout_s
        self.reacquire_timeout_s = reacquire_timeout_s
        self.state = TrackingState.APPROACH
        self.last_track_time_s: float | None = None

    def decide(
        self,
        has_target: bool,
        telemetry_valid: bool,
        bbox_valid: bool,
        current_time_s: float,
    ) -> TrackingState:
        if not telemetry_valid or not has_target:
            self.state = TrackingState.FAILSAFE
            return self.state

        if bbox_valid:
            self.last_track_time_s = current_time_s
            self.state = TrackingState.TRACK
            return self.state

        if self.last_track_time_s is None:
            self.state = TrackingState.APPROACH
            return self.state

        lost_duration_s = current_time_s - self.last_track_time_s
        if lost_duration_s <= self.lost_timeout_s:
            self.state = TrackingState.LOST
        elif lost_duration_s <= self.reacquire_timeout_s:
            self.state = TrackingState.APPROACH
        else:
            self.state = TrackingState.FAILSAFE
        return self.state
