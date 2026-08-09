#!/usr/bin/env python3
"""
FS Interface - fs_adapter. Sits upstream of vehicle_controller, translating
external ROS/MQTT commands into the (condition_name, token, command) shape
vehicle_controller expects, and receiving back the outcome of whatever it
submits via on_result().

No thread of its own: whatever eventually updates the pending command (a ROS
subscription callback) and vehicle_controller's tick that reads it both run on
the same single executor thread, so there's no concurrent writer to guard
against - this is a plain synchronous read, not an atomic cross-thread one.

Stub for now: never has anything pending, always ('FS_ADAPTER', 0, None) -
never sends a command - just enough for the surrounding pipeline to compile
and run end-to-end before any real external command intake exists.
"""

from typing import Any, Tuple


class FsInterface:

    def __init__(self, mavros):
        self._mavros = mavros

    def fs_adapter_atomic(self) -> Tuple[Any, Any, Any]:
        return ('FS_ADAPTER', 0, None)

    def report(self, cmd_status, condition_name, command, token, reason):
        pass  # stub

    def publish_telemetry(self):
        """Called once per vc tick. Stub - will actually publish out (ROS/MQTT)
        later; for now just pulls the current snapshot and does nothing with it."""
        telemetry = self._mavros.get_telemetry()
