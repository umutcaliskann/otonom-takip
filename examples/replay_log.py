"""Replay a JSONL tracking log.

Run:
    python3 -m examples.replay_log path/to/flight.jsonl
"""

from __future__ import annotations

import sys

from autonomous_tracking import PipelineConfig
from autonomous_tracking.replay import ReplayRunner


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python3 -m examples.replay_log path/to/flight.jsonl")
    runner = ReplayRunner.from_config(PipelineConfig.from_file("config/default.yaml"))
    summary = runner.replay_file(sys.argv[1])
    print(summary)


if __name__ == "__main__":
    main()
