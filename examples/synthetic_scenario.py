"""Synthetic scenario for the autonomous tracking pipeline.

Run:
    python3 -m examples.synthetic_scenario
"""

from autonomous_tracking import (
    AutonomousTrackingPipeline,
    CameraIntrinsics,
    PipelineConfig,
    TargetBBox,
    TargetTelemetry,
    UavTelemetry,
)


def main() -> None:
    camera = CameraIntrinsics.from_fov(
        image_width_px=1280,
        image_height_px=720,
        horizontal_fov_deg=70.0,
        vertical_fov_deg=43.0,
    )
    pipeline = AutonomousTrackingPipeline(PipelineConfig(camera=camera))

    ego = UavTelemetry(
        latitude_deg=39.9250,
        longitude_deg=32.8369,
        altitude_m=1_050.0,
        speed_mps=23.0,
        heading_deg=85.0,
        timestamp_s=0.0,
    )
    targets = [
        TargetTelemetry("rakip-1", 39.9261, 32.8425, 1_080.0, 21.0, 250.0, 0.0),
        TargetTelemetry("rakip-2", 39.9248, 32.8395, 1_450.0, 20.0, 180.0, 0.0),
    ]

    approach_status = pipeline.process(ego, targets, current_time_s=0.4)
    print("Telemetry approach status:")
    print(approach_status)

    bbox = TargetBBox(
        x_px=714.0,
        y_px=288.0,
        width_px=92.0,
        height_px=54.0,
        timestamp_s=0.1,
        confidence=1.0,
    )
    visual_status = pipeline.process(
        ego,
        targets,
        bbox,
        current_time_s=0.5,
    )
    print("\nVisual PN guidance status:")
    print(visual_status)


if __name__ == "__main__":
    main()
