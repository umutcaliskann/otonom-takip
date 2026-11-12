"""Replay JSONL tracking logs and summarize behavior."""

from __future__ import annotations

import json
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .models import TargetBBox, TargetTelemetry, TrackingState, UavTelemetry
from .pipeline import AutonomousTrackingPipeline, PipelineConfig


@dataclass(frozen=True)
class ReplaySummary:
    processed_records: int
    skipped_records: int
    state_counts: dict[str, int]
    command_count: int


class ReplayRunner:
    def __init__(self, pipeline: AutonomousTrackingPipeline) -> None:
        self.pipeline = pipeline

    @classmethod
    def from_config(cls, config: PipelineConfig) -> "ReplayRunner":
        return cls(AutonomousTrackingPipeline(config))

    def replay_file(self, path: str | Path) -> ReplaySummary:
        state_counts: Counter[str] = Counter()
        processed = 0
        skipped = 0
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                skipped += 1
                continue
            try:
                state = self._process_record(record)
            except (KeyError, TypeError, ValueError):
                skipped += 1
                continue
            state_counts[state] += 1
            processed += 1
        return ReplaySummary(
            processed_records=processed,
            skipped_records=skipped,
            state_counts=dict(state_counts),
            command_count=processed,
        )

    def _process_record(self, record: dict[str, Any]) -> str:
        if "ego" not in record or "targets" not in record:
            return str(record["state"])
        ego = UavTelemetry(**record["ego"])
        targets = [TargetTelemetry(**target) for target in record["targets"]]
        bbox = TargetBBox(**record["bbox"]) if record.get("bbox") is not None else None
        status = self.pipeline.process(
            ego,
            targets,
            bbox,
            current_time_s=float(record["timestamp_s"]),
        )
        state = status.state
        return state.value if isinstance(state, TrackingState) else str(state)
