"""Autopilot adapter interfaces.

This module intentionally does not send MAVLink/PX4 messages. It provides the
boundary where a real integration can be attached later.
"""

from __future__ import annotations

from typing import Protocol

from .models import GuidanceCommand, TrackingStatus


class AutopilotAdapter(Protocol):
    def send(self, status: TrackingStatus) -> GuidanceCommand:
        ...


class DryRunAutopilotAdapter:
    def __init__(self) -> None:
        self.sent_commands: list[GuidanceCommand] = []

    def send(self, status: TrackingStatus) -> GuidanceCommand:
        command = status.command
        self.sent_commands.append(command)
        return command
