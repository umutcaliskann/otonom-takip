"""Run a longer synthetic tracking scenario."""

from collections import Counter

from autonomous_tracking import AutonomousTrackingPipeline, PipelineConfig
from autonomous_tracking.simulation import SyntheticScenario


def main() -> None:
    pipeline = AutonomousTrackingPipeline(PipelineConfig.from_file("config/default.yaml"))
    statuses = SyntheticScenario(pipeline, duration_s=30.0, dt_s=0.2).run()
    counts = Counter(status.state.value for status in statuses)
    print("Synthetic simulation state counts:")
    print(dict(counts))
    print("Last status:")
    print(statuses[-1])


if __name__ == "__main__":
    main()
