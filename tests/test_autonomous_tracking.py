import math
import tempfile
from pathlib import Path
import unittest
import json

from autonomous_tracking import (
    AutonomousTrackingPipeline,
    CameraIntrinsics,
    DryRunAutopilotAdapter,
    LineOfSightEKF,
    LineOfSightState,
    PipelineConfig,
    ProportionalNavigationGuidance,
    SyntheticScenario,
    TargetBBox,
    TargetLockManager,
    TargetTelemetry,
    TrackingState,
    UavTelemetry,
    VisualMeasurement,
    load_camera_intrinsics,
)
from autonomous_tracking.command_smoothing import CommandSmoother
from autonomous_tracking.safety import SafetySupervisor
from autonomous_tracking.replay import ReplayRunner
from autonomous_tracking.telemetry_compensation import DelayedTelemetryCompensator
from autonomous_tracking.target_selection import TargetSelector
from autonomous_tracking.validation import InputValidator
from autonomous_tracking.vision_geometry import los_from_visual_measurement


class AutonomousTrackingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.ego = UavTelemetry(39.0, 32.0, 1_000.0, 22.0, 90.0, 0.0)
        self.camera = CameraIntrinsics.from_fov(1280, 720, 70.0, 43.0)

    def test_target_selector_uses_3d_distance(self) -> None:
        close_but_high = TargetTelemetry("high", 39.0001, 32.0001, 2_000.0, 20.0, 0.0, 0.0)
        farther_level = TargetTelemetry("level", 39.0010, 32.0010, 1_010.0, 20.0, 0.0, 0.0)
        result = TargetSelector().select(self.ego, [close_but_high, farther_level])
        self.assertIsNotNone(result)
        self.assertEqual(result.target.target_id, "level")

    def test_los_center_is_near_zero(self) -> None:
        visual = VisualMeasurement(640.0, 360.0, 80.0, 40.0, 0.9, 1.0)
        los = los_from_visual_measurement(visual, self.camera)
        self.assertAlmostEqual(los.azimuth_rad, 0.0, places=6)
        self.assertAlmostEqual(los.elevation_rad, 0.0, places=6)

    def test_los_signs_match_image_position(self) -> None:
        visual = VisualMeasurement(740.0, 310.0, 80.0, 40.0, 0.9, 1.0)
        los = los_from_visual_measurement(visual, self.camera)
        self.assertGreater(los.azimuth_rad, 0.0)
        self.assertGreater(los.elevation_rad, 0.0)

    def test_target_bbox_converts_to_center_measurement(self) -> None:
        bbox = TargetBBox(x_px=600.0, y_px=330.0, width_px=80.0, height_px=60.0)
        visual = bbox.to_visual_measurement()
        self.assertEqual(visual.bbox_center_x_px, 640.0)
        self.assertEqual(visual.bbox_center_y_px, 360.0)
        self.assertEqual(visual.confidence, 1.0)

    def test_delayed_telemetry_compensation_projects_forward(self) -> None:
        compensator = DelayedTelemetryCompensator(max_prediction_dt_s=2.0)
        ego = UavTelemetry(39.0, 32.0, 1_000.0, 20.0, 90.0, 0.0)
        predicted = compensator.compensate_ego(ego, current_time_s=1.0)
        self.assertGreater(predicted.longitude_deg, ego.longitude_deg)
        self.assertAlmostEqual(predicted.latitude_deg, ego.latitude_deg, places=3)
        self.assertEqual(predicted.timestamp_s, 1.0)

    def test_ekf_smooths_noisy_measurements(self) -> None:
        ekf = LineOfSightEKF(process_noise=0.01, measurement_noise=0.05)
        measurements = [
            VisualMeasurement(700.0, 360.0, 80.0, 40.0, 0.9, 0.0),
            VisualMeasurement(760.0, 360.0, 80.0, 40.0, 0.9, 0.1),
            VisualMeasurement(680.0, 360.0, 80.0, 40.0, 0.9, 0.2),
        ]
        raw = [los_from_visual_measurement(m, self.camera) for m in measurements]
        states = [ekf.update(m) for m in raw]
        raw_jump = abs(raw[-1].azimuth_rad - raw[-2].azimuth_rad)
        filtered_jump = abs(states[-1].azimuth_rad - states[-2].azimuth_rad)
        self.assertLess(filtered_jump, raw_jump)
        self.assertTrue(math.isfinite(states[-1].azimuth_rate_rad_s))

    def test_pn_guidance_signs(self) -> None:
        guidance = ProportionalNavigationGuidance()
        target = TargetTelemetry("target", 39.001, 32.001, 1_000.0, 20.0, 0.0, 0.0)
        state = LineOfSightState(azimuth_rad=0.1, elevation_rad=0.05, azimuth_rate_rad_s=0.0, elevation_rate_rad_s=0.0, timestamp_s=1.0)
        command = guidance.command(self.ego, target, state, airspeed_mps=24.0)
        self.assertGreater(command.roll_deg, 0.0)
        self.assertGreater(command.pitch_deg, 0.0)
        self.assertEqual(command.mode, "GUIDED")

    def test_pipeline_approach_without_visual(self) -> None:
        pipeline = AutonomousTrackingPipeline(PipelineConfig(camera=self.camera))
        target = TargetTelemetry("target", 39.001, 32.001, 1_020.0, 20.0, 0.0, 0.0)
        status = pipeline.process(self.ego, [target])
        self.assertEqual(status.state, TrackingState.APPROACH)
        self.assertEqual(status.command.source, "telemetry_approach")

    def test_pipeline_visual_guidance(self) -> None:
        pipeline = AutonomousTrackingPipeline(PipelineConfig(camera=self.camera))
        target = TargetTelemetry("target", 39.001, 32.001, 1_020.0, 20.0, 0.0, 0.0)
        bbox = TargetBBox(705.0, 295.0, 90.0, 50.0, 1.0)
        status = pipeline.process(self.ego, [target], bbox, current_time_s=1.0)
        self.assertEqual(status.state, TrackingState.TRACK)
        self.assertEqual(status.command.source, "visual_pn_guidance")
        self.assertEqual(status.command.mode, "GUIDED")
        self.assertEqual(status.command.timestamp_s, 1.0)

    def test_bbox_validation_rejects_bad_inputs(self) -> None:
        validator = InputValidator(min_bbox_area_ratio=0.001)
        self.assertFalse(
            validator.validate_bbox(TargetBBox(10.0, 10.0, -1.0, 20.0), self.camera, 0.0).is_valid
        )
        self.assertFalse(
            validator.validate_bbox(TargetBBox(2000.0, 10.0, 20.0, 20.0), self.camera, 0.0).is_valid
        )
        self.assertFalse(
            validator.validate_bbox(TargetBBox(10.0, 10.0, 2.0, 2.0), self.camera, 0.0).is_valid
        )
        self.assertTrue(
            validator.validate_bbox(TargetBBox(600.0, 330.0, 80.0, 60.0), self.camera, 0.0).is_valid
        )

    def test_stale_telemetry_returns_failsafe(self) -> None:
        pipeline = AutonomousTrackingPipeline(PipelineConfig(camera=self.camera))
        target = TargetTelemetry("target", 39.001, 32.001, 1_020.0, 20.0, 0.0, 0.0)
        status = pipeline.process(self.ego, [target], current_time_s=5.0)
        self.assertEqual(status.state, TrackingState.FAILSAFE)
        self.assertEqual(status.reason, "stale_telemetry")
        self.assertIn("failsafe", status.command.source)

    def test_short_bbox_loss_enters_lost_state(self) -> None:
        pipeline = AutonomousTrackingPipeline(PipelineConfig(camera=self.camera, lost_timeout_s=1.0))
        target = TargetTelemetry("target", 39.001, 32.001, 1_020.0, 20.0, 0.0, 0.0)
        bbox = TargetBBox(705.0, 295.0, 90.0, 50.0, 0.1)
        track_status = pipeline.process(self.ego, [target], bbox, current_time_s=0.1)
        self.assertEqual(track_status.state, TrackingState.TRACK)
        lost_status = pipeline.process(self.ego, [target], current_time_s=0.5)
        self.assertEqual(lost_status.state, TrackingState.LOST)
        self.assertEqual(lost_status.command.source, "lost_telemetry_approach")

    def test_command_smoothing_limits_delta(self) -> None:
        smoother = CommandSmoother(max_delta_roll_deg=5.0, max_delta_pitch_deg=4.0, max_delta_airspeed_mps=2.0)
        target = TargetTelemetry("target", 39.001, 32.001, 1_000.0, 20.0, 0.0, 0.0)
        first = ProportionalNavigationGuidance().command(
            self.ego,
            target,
            LineOfSightState(0.0, 0.0, 0.0, 0.0, 0.0),
            20.0,
        )
        second = ProportionalNavigationGuidance().command(
            self.ego,
            target,
            LineOfSightState(1.0, 1.0, 0.0, 0.0, 0.1),
            30.0,
        )
        smoother.smooth(first)
        smoothed = smoother.smooth(second)
        self.assertLessEqual(abs(smoothed.roll_deg - first.roll_deg), 5.0)
        self.assertLessEqual(abs(smoothed.pitch_deg - first.pitch_deg), 4.0)
        self.assertLessEqual(abs(smoothed.airspeed_mps - first.airspeed_mps), 2.0)

    def test_safety_limits_command(self) -> None:
        supervisor = SafetySupervisor(max_roll_deg=20.0, max_pitch_deg=10.0, max_airspeed_mps=25.0)
        target = TargetTelemetry("target", 39.001, 32.001, 1_000.0, 20.0, 0.0, 0.0)
        command = ProportionalNavigationGuidance(max_roll_deg=90.0, max_pitch_deg=90.0).command(
            self.ego,
            target,
            LineOfSightState(2.0, 2.0, 0.0, 0.0, 0.0),
            40.0,
        )
        limited = supervisor.apply_limits(command)
        self.assertEqual(limited.roll_deg, 20.0)
        self.assertEqual(limited.pitch_deg, 10.0)
        self.assertEqual(limited.airspeed_mps, 25.0)

    def test_pipeline_writes_jsonl_log(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = Path(tmpdir) / "flight.jsonl"
            pipeline = AutonomousTrackingPipeline(PipelineConfig(camera=self.camera, log_path=log_path))
            target = TargetTelemetry("target", 39.001, 32.001, 1_020.0, 20.0, 0.0, 0.0)
            pipeline.process(self.ego, [target], current_time_s=0.0)
            lines = log_path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 1)
            self.assertIn('"state": "APPROACH"', lines[0])
            record = json.loads(lines[0])
            self.assertIn("ego", record)
            self.assertIn("targets", record)

    def test_default_config_loads_pipeline_config(self) -> None:
        config = PipelineConfig.from_file("config/default.yaml")
        self.assertEqual(config.camera.image_width_px, 1280)
        self.assertEqual(config.max_telemetry_age_s, 2.0)

    def test_camera_config_supports_fov_and_rejects_invalid_values(self) -> None:
        camera = load_camera_intrinsics(
            {
                "image_width_px": 1280,
                "image_height_px": 720,
                "horizontal_fov_deg": 70.0,
                "vertical_fov_deg": 43.0,
            }
        )
        self.assertAlmostEqual(camera.focal_length_x_px, self.camera.focal_length_x_px)
        with self.assertRaises(ValueError):
            load_camera_intrinsics({"image_width_px": 1280, "image_height_px": 720, "horizontal_fov_deg": 0.0})

    def test_target_lock_keeps_existing_target_until_switch_confirmed(self) -> None:
        lock = TargetLockManager(switch_distance_ratio=0.8, switch_confirmations=2)
        selector = TargetSelector()
        locked = TargetTelemetry("locked", 39.002, 32.002, 1_000.0, 20.0, 0.0, 0.0)
        better = TargetTelemetry("better", 39.0001, 32.0001, 1_000.0, 20.0, 0.0, 0.0)
        first = lock.select(self.ego, [locked], selector)
        self.assertEqual(first.target.target_id, "locked")
        second = lock.select(self.ego, [locked, better], selector)
        self.assertEqual(second.target.target_id, "locked")
        third = lock.select(self.ego, [locked, better], selector)
        self.assertEqual(third.target.target_id, "better")

    def test_replay_reads_log_and_skips_bad_lines(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = Path(tmpdir) / "flight.jsonl"
            pipeline = AutonomousTrackingPipeline(PipelineConfig(camera=self.camera, log_path=log_path))
            target = TargetTelemetry("target", 39.001, 32.001, 1_020.0, 20.0, 0.0, 0.0)
            pipeline.process(self.ego, [target], current_time_s=0.0)
            with log_path.open("a", encoding="utf-8") as file:
                file.write("not-json\n")
            summary = ReplayRunner.from_config(PipelineConfig(camera=self.camera)).replay_file(log_path)
            self.assertEqual(summary.processed_records, 1)
            self.assertEqual(summary.skipped_records, 1)
            self.assertIn("APPROACH", summary.state_counts)

    def test_long_synthetic_simulation_exercises_states(self) -> None:
        pipeline = AutonomousTrackingPipeline(PipelineConfig(camera=self.camera))
        statuses = SyntheticScenario(pipeline, duration_s=10.0, dt_s=0.2).run()
        states = {status.state for status in statuses}
        self.assertIn(TrackingState.APPROACH, states)
        self.assertIn(TrackingState.TRACK, states)
        self.assertIn(TrackingState.LOST, states)

    def test_dry_run_autopilot_adapter_records_command(self) -> None:
        pipeline = AutonomousTrackingPipeline(PipelineConfig(camera=self.camera))
        target = TargetTelemetry("target", 39.001, 32.001, 1_020.0, 20.0, 0.0, 0.0)
        status = pipeline.process(self.ego, [target], current_time_s=0.0)
        adapter = DryRunAutopilotAdapter()
        command = adapter.send(status)
        self.assertEqual(command, status.command)
        self.assertEqual(len(adapter.sent_commands), 1)


if __name__ == "__main__":
    unittest.main()
