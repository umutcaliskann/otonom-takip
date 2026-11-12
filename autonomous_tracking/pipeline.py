"""High-level autonomous tracking pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Iterable, Optional, Union

from .approach import ApproachController
from .camera_calibration import load_camera_intrinsics
from .command_smoothing import CommandSmoother
from .config import load_pipeline_config
from .ekf import LineOfSightEKF
from .geo import distance_3d_m
from .logging import FlightLogger
from .models import (
    CameraIntrinsics,
    GuidanceCommand,
    TargetBBox,
    TargetTelemetry,
    TrackingState,
    TrackingStatus,
    UavTelemetry,
    VisualMeasurement,
)
from .pn_guidance import ProportionalNavigationGuidance
from .safety import SafetySupervisor
from .speed_control import SpeedController
from .state_machine import TrackingStateMachine
from .target_lock import TargetLockManager
from .target_selection import TargetSelector
from .telemetry_compensation import DelayedTelemetryCompensator
from .validation import InputValidator, ValidationResult
from .vision_geometry import los_from_visual_measurement


@dataclass
class PipelineConfig:
    camera: CameraIntrinsics
    min_visual_confidence: float = 0.45
    compensate_delayed_telemetry: bool = True
    max_telemetry_age_s: float = 2.0
    max_bbox_age_s: float = 0.5
    lost_timeout_s: float = 1.0
    reacquire_timeout_s: float = 3.0
    min_bbox_area_ratio: float = 0.0001
    max_command_delta_roll_deg: float = 8.0
    max_command_delta_pitch_deg: float = 5.0
    max_command_delta_airspeed_mps: float = 3.0
    log_path: Optional[Union[str, Path]] = None
    selector: TargetSelector = field(default_factory=TargetSelector)
    approach: ApproachController = field(default_factory=ApproachController)
    ekf: LineOfSightEKF = field(default_factory=LineOfSightEKF)
    guidance: ProportionalNavigationGuidance = field(default_factory=ProportionalNavigationGuidance)
    speed: SpeedController = field(default_factory=SpeedController)
    safety: SafetySupervisor = field(default_factory=SafetySupervisor)
    target_lock: TargetLockManager = field(default_factory=TargetLockManager)
    telemetry_compensator: DelayedTelemetryCompensator = field(
        default_factory=DelayedTelemetryCompensator
    )

    @classmethod
    def from_file(cls, path: Union[str, Path]) -> "PipelineConfig":
        data = load_pipeline_config(path)
        camera = load_camera_intrinsics(data.get("camera", {}))
        guidance_data = data.get("guidance", {})
        speed_data = data.get("speed", {})
        safety_data = data.get("safety", {})
        smoothing_data = data.get("smoothing", {})
        validation_data = data.get("validation", {})
        state_data = data.get("state_machine", {})
        target_lock_data = data.get("target_lock", {})
        telemetry_data = data.get("telemetry_compensation", {})
        return cls(
            camera=camera,
            min_visual_confidence=float(validation_data.get("min_visual_confidence", 0.45)),
            compensate_delayed_telemetry=bool(
                telemetry_data.get("enabled", data.get("compensate_delayed_telemetry", True))
            ),
            max_telemetry_age_s=float(validation_data.get("max_telemetry_age_s", 2.0)),
            max_bbox_age_s=float(validation_data.get("max_bbox_age_s", 0.5)),
            lost_timeout_s=float(state_data.get("lost_timeout_s", 1.0)),
            reacquire_timeout_s=float(state_data.get("reacquire_timeout_s", 3.0)),
            min_bbox_area_ratio=float(validation_data.get("min_bbox_area_ratio", 0.0001)),
            max_command_delta_roll_deg=float(smoothing_data.get("max_delta_roll_deg", 8.0)),
            max_command_delta_pitch_deg=float(smoothing_data.get("max_delta_pitch_deg", 5.0)),
            max_command_delta_airspeed_mps=float(
                smoothing_data.get("max_delta_airspeed_mps", 3.0)
            ),
            log_path=data.get("logging", {}).get("path"),
            guidance=ProportionalNavigationGuidance(
                navigation_constant=float(guidance_data.get("navigation_constant", 3.0)),
                los_angle_gain=float(guidance_data.get("los_angle_gain", 70.0)),
                los_rate_gain=float(guidance_data.get("los_rate_gain", 28.0)),
                max_roll_deg=float(guidance_data.get("max_roll_deg", 35.0)),
                max_pitch_deg=float(guidance_data.get("max_pitch_deg", 15.0)),
                max_yaw_rate_deg_s=float(guidance_data.get("max_yaw_rate_deg_s", 25.0)),
            ),
            speed=SpeedController(
                min_airspeed_mps=float(speed_data.get("min_airspeed_mps", 16.0)),
                cruise_airspeed_mps=float(speed_data.get("cruise_airspeed_mps", 22.0)),
                max_airspeed_mps=float(speed_data.get("max_airspeed_mps", 30.0)),
                far_distance_m=float(speed_data.get("far_distance_m", 500.0)),
                near_distance_m=float(speed_data.get("near_distance_m", 90.0)),
                small_bbox_ratio=float(speed_data.get("small_bbox_ratio", 0.004)),
                large_bbox_ratio=float(speed_data.get("large_bbox_ratio", 0.08)),
            ),
            safety=SafetySupervisor(
                max_roll_deg=float(safety_data.get("max_roll_deg", 35.0)),
                max_pitch_deg=float(safety_data.get("max_pitch_deg", 15.0)),
                max_yaw_rate_deg_s=float(safety_data.get("max_yaw_rate_deg_s", 25.0)),
                min_airspeed_mps=float(safety_data.get("min_airspeed_mps", 16.0)),
                max_airspeed_mps=float(safety_data.get("max_airspeed_mps", 30.0)),
                safe_airspeed_mps=float(safety_data.get("safe_airspeed_mps", 22.0)),
                close_distance_m=float(safety_data.get("close_distance_m", 45.0)),
            ),
            target_lock=TargetLockManager(
                switch_distance_ratio=float(target_lock_data.get("switch_distance_ratio", 0.75)),
                switch_confirmations=int(target_lock_data.get("switch_confirmations", 2)),
            ),
            telemetry_compensator=DelayedTelemetryCompensator(
                max_prediction_dt_s=float(telemetry_data.get("max_prediction_dt_s", 2.0))
            ),
        )


class AutonomousTrackingPipeline:
    def __init__(self, config: PipelineConfig) -> None:
        self.config = config
        self.validator = InputValidator(
            max_telemetry_age_s=config.max_telemetry_age_s,
            max_bbox_age_s=config.max_bbox_age_s,
            min_bbox_area_ratio=config.min_bbox_area_ratio,
        )
        self.state_machine = TrackingStateMachine(
            lost_timeout_s=config.lost_timeout_s,
            reacquire_timeout_s=config.reacquire_timeout_s,
        )
        self.safety = config.safety
        self.target_lock = config.target_lock
        self.smoother = CommandSmoother(
            max_delta_roll_deg=config.max_command_delta_roll_deg,
            max_delta_pitch_deg=config.max_command_delta_pitch_deg,
            max_delta_airspeed_mps=config.max_command_delta_airspeed_mps,
        )
        self.logger = FlightLogger(config.log_path) if config.log_path is not None else None

    def process(
        self,
        ego: UavTelemetry,
        targets: Iterable[TargetTelemetry],
        visual: Optional[Union[TargetBBox, VisualMeasurement]] = None,
        current_time_s: Optional[float] = None,
    ) -> TrackingStatus:
        timestamp_s = self._command_timestamp(ego, visual, current_time_s)
        visual_bbox = self._visual_to_bbox(visual)
        target_candidates = list(targets)
        bbox_validation = self.validator.validate_bbox(
            visual_bbox,
            self.config.camera,
            timestamp_s,
        )
        ego_validation = self.validator.validate_ego_telemetry(ego, timestamp_s)
        valid_targets = [
            target
            for target in target_candidates
            if self.validator.validate_target_telemetry(target, timestamp_s).is_valid
        ]

        if not ego_validation.is_valid or not valid_targets:
            reason = ego_validation.reason or "missing_target_telemetry"
            command = self.safety.failsafe_command(ego, timestamp_s, reason)
            status = self._status(
                TrackingState.FAILSAFE,
                None,
                command,
                reason,
                0.0,
                timestamp_s,
                bbox_validation,
            )
            self._log(status, bbox_validation, ego, target_candidates, visual_bbox)
            return status

        ego_for_guidance = ego
        targets_for_guidance = valid_targets
        if self.config.compensate_delayed_telemetry:
            ego_for_guidance = self.config.telemetry_compensator.compensate_ego(
                ego,
                timestamp_s,
            )
            targets_for_guidance = self.config.telemetry_compensator.compensate_targets(
                targets_for_guidance,
                timestamp_s,
            )

        selection = self.target_lock.select(
            ego_for_guidance,
            targets_for_guidance,
            self.config.selector,
        )
        if selection is None:
            reason = "no_selectable_target"
            command = self.safety.failsafe_command(ego_for_guidance, timestamp_s, reason)
            status = self._status(
                TrackingState.FAILSAFE,
                None,
                command,
                reason,
                0.0,
                timestamp_s,
                bbox_validation,
            )
            self._log(status, bbox_validation, ego, target_candidates, visual_bbox)
            return status

        target = selection.target
        visual_measurement = self._normalize_visual_input(visual)
        bbox_is_usable = (
            bbox_validation.is_valid
            and visual_measurement is not None
            and visual_measurement.confidence >= self.config.min_visual_confidence
        )
        state = self.state_machine.decide(
            has_target=True,
            telemetry_valid=True,
            bbox_valid=bbox_is_usable,
            current_time_s=timestamp_s,
        )

        if state == TrackingState.FAILSAFE:
            reason = bbox_validation.reason or "target_lost_timeout"
            command = self.safety.failsafe_command(ego_for_guidance, timestamp_s, reason)
            status = self._status(
                state,
                target.target_id,
                command,
                reason,
                bbox_validation.quality,
                timestamp_s,
                bbox_validation,
            )
            self._log(status, bbox_validation, ego, target_candidates, visual_bbox)
            return status

        if state in (TrackingState.APPROACH, TrackingState.LOST):
            command = self.config.approach.command(
                ego=ego_for_guidance,
                target=target,
                timestamp_s=timestamp_s,
            )
            command = self.safety.apply_limits(command, selection.distance_3d_m)
            command = self.smoother.smooth(command)
            if state == TrackingState.LOST:
                command = replace(command, source="lost_telemetry_approach")
            status = self._status(
                state,
                target.target_id,
                command,
                bbox_validation.reason,
                bbox_validation.quality,
                timestamp_s,
                bbox_validation,
            )
            self._log(status, bbox_validation, ego, target_candidates, visual_bbox)
            return status

        assert visual_measurement is not None
        los_measurement = los_from_visual_measurement(visual_measurement, self.config.camera)
        los_state = self.config.ekf.update(los_measurement)
        distance = distance_3d_m(ego_for_guidance, target)
        airspeed = self.config.speed.command_speed(
            distance,
            visual_measurement,
            self.config.camera,
        )
        command = self.config.guidance.command(
            ego=ego_for_guidance,
            target=target,
            los=los_state,
            airspeed_mps=airspeed,
        )
        command = replace(command, timestamp_s=timestamp_s)
        command = self.safety.apply_limits(command, distance)
        command = self.smoother.smooth(command)
        status = self._status(
            TrackingState.TRACK,
            target.target_id,
            command,
            None,
            bbox_validation.quality,
            timestamp_s,
            bbox_validation,
        )
        self._log(status, bbox_validation, ego, target_candidates, visual_bbox)
        return status

    def _normalize_visual_input(
        self,
        visual: Optional[Union[TargetBBox, VisualMeasurement]],
    ) -> Optional[VisualMeasurement]:
        if visual is None:
            return None
        if isinstance(visual, TargetBBox):
            return visual.to_visual_measurement()
        return visual

    def _visual_to_bbox(
        self,
        visual: Optional[Union[TargetBBox, VisualMeasurement]],
    ) -> Optional[TargetBBox]:
        if visual is None:
            return None
        if isinstance(visual, TargetBBox):
            return visual
        return TargetBBox(
            x_px=visual.bbox_center_x_px - visual.bbox_width_px / 2.0,
            y_px=visual.bbox_center_y_px - visual.bbox_height_px / 2.0,
            width_px=visual.bbox_width_px,
            height_px=visual.bbox_height_px,
            timestamp_s=visual.timestamp_s,
            confidence=visual.confidence,
        )

    def _command_timestamp(
        self,
        ego: UavTelemetry,
        visual: Optional[Union[TargetBBox, VisualMeasurement]],
        current_time_s: Optional[float],
    ) -> float:
        if current_time_s is not None:
            return current_time_s
        if visual is not None:
            return visual.timestamp_s
        return ego.timestamp_s

    def _status(
        self,
        state: TrackingState,
        target_id: Optional[str],
        command: GuidanceCommand,
        reason: Optional[str],
        measurement_quality: float,
        timestamp_s: float,
        validation: ValidationResult,
    ) -> TrackingStatus:
        return TrackingStatus(
            state=state,
            target_id=target_id,
            command=command,
            reason=reason or validation.reason,
            measurement_quality=measurement_quality,
            timestamp_s=timestamp_s,
        )

    def _log(
        self,
        status: TrackingStatus,
        bbox_validation: ValidationResult,
        ego: UavTelemetry,
        targets: list[TargetTelemetry],
        bbox: Optional[TargetBBox],
    ) -> None:
        if self.logger is None:
            return
        self.logger.log(
            {
                "timestamp_s": status.timestamp_s,
                "ego": ego,
                "targets": targets,
                "bbox": bbox,
                "state": status.state,
                "target_id": status.target_id,
                "command": status.command,
                "reason": status.reason,
                "measurement_quality": status.measurement_quality,
                "bbox_validation": bbox_validation,
            }
        )
