"""Autonomous target tracking and fixed-wing guidance toolkit."""

from .approach import ApproachController
from .autopilot_adapter import DryRunAutopilotAdapter
from .camera_calibration import load_camera_intrinsics
from .ekf import LineOfSightEKF, LineOfSightState
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
from .pipeline import AutonomousTrackingPipeline, PipelineConfig
from .pn_guidance import ProportionalNavigationGuidance
from .replay import ReplayRunner, ReplaySummary
from .speed_control import SpeedController
from .simulation import SyntheticScenario
from .target_lock import TargetLockManager
from .target_selection import TargetSelectionResult, TargetSelector

__all__ = [
    "ApproachController",
    "AutonomousTrackingPipeline",
    "CameraIntrinsics",
    "DryRunAutopilotAdapter",
    "GuidanceCommand",
    "LineOfSightEKF",
    "LineOfSightState",
    "PipelineConfig",
    "ProportionalNavigationGuidance",
    "ReplayRunner",
    "ReplaySummary",
    "SpeedController",
    "SyntheticScenario",
    "TargetBBox",
    "TargetLockManager",
    "TargetSelectionResult",
    "TargetSelector",
    "TargetTelemetry",
    "TrackingState",
    "TrackingStatus",
    "UavTelemetry",
    "VisualMeasurement",
    "load_camera_intrinsics",
]
