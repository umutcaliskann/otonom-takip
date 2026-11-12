"""YAML configuration loading."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "default.yaml"


def load_pipeline_config(path: str | Path | None = None) -> dict[str, Any]:
    config_path = DEFAULT_CONFIG_PATH if path is None else Path(path)
    with config_path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    if not isinstance(data, dict):
        raise ValueError("Pipeline config must be a YAML mapping.")
    return data
