#!/usr/bin/env python3

"""Persistent runtime state for startup/shutdown safety."""

from __future__ import annotations

import json
import os
import tempfile
import time
from typing import Iterable, Optional


class RuntimeStateManager:
    """Stores whether the platform is safely referenced for motion."""

    DEFAULT_FILENAME = "motor_runtime_state.json"

    def __init__(self, state_path: Optional[str] = None):
        self.state_path = state_path or self._default_state_path()

    @staticmethod
    def _default_state_path():
        runtime_dir = os.environ.get(
            "STEWART_RUNTIME_DIR",
            os.path.join(os.path.expanduser("~"), ".stewart_control"),
        )
        return os.path.join(runtime_dir, RuntimeStateManager.DEFAULT_FILENAME)

    def _default_state(self):
        now = time.strftime("%Y-%m-%dT%H:%M:%S")
        return {
            "version": 1,
            "position_known": False,
            "home_confirmed": False,
            "recovery_required": True,
            "clean_shutdown": False,
            "shutdown_in_progress": False,
            "session_dirty": False,
            "owner_pid": None,
            "last_feedback_cm": None,
            "last_command_cm": None,
            "last_update": now,
            "last_status_message": "Recovery required before motion.",
        }

    @staticmethod
    def _pid_is_alive(pid):
        if pid in (None, 0):
            return False
        try:
            os.kill(int(pid), 0)
        except (OSError, ValueError, TypeError):
            return False
        return True

    def _normalize_state(self, state):
        if (
            state.get("session_dirty")
            and state.get("owner_pid") is not None
            and not self._pid_is_alive(state.get("owner_pid"))
        ):
            state["position_known"] = False
            state["home_confirmed"] = False
            state["recovery_required"] = True
            state["clean_shutdown"] = False
            state["shutdown_in_progress"] = False
            state["session_dirty"] = False
            state["owner_pid"] = None
            state["last_status_message"] = (
                "Previous session ended unexpectedly. Recovery required before motion."
            )
            return True
        return False

    def load(self):
        if not os.path.exists(self.state_path):
            return self._default_state()
        try:
            with open(self.state_path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except Exception:
            return self._default_state()

        default = self._default_state()
        default.update(data)
        if self._normalize_state(default):
            self.save(default)
        return default

    def save(self, state):
        os.makedirs(os.path.dirname(self.state_path), exist_ok=True)
        state["last_update"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        with tempfile.NamedTemporaryFile(
            "w",
            delete=False,
            dir=os.path.dirname(self.state_path),
            encoding="utf-8",
        ) as handle:
            json.dump(state, handle, indent=2)
            temp_path = handle.name
        os.replace(temp_path, self.state_path)

    @staticmethod
    def _vector_or_none(values: Optional[Iterable[float]]):
        if values is None:
            return None
        return [round(float(value), 4) for value in values]

    def begin_motion_session(self):
        state = self.load()
        trusted = bool(
            not state.get("recovery_required", True)
            and state.get("position_known")
            and state.get("home_confirmed")
            and state.get("clean_shutdown")
        )
        state["position_known"] = trusted
        state["home_confirmed"] = trusted
        state["recovery_required"] = not trusted
        state["clean_shutdown"] = False
        state["shutdown_in_progress"] = False
        state["session_dirty"] = True
        state["owner_pid"] = os.getpid()
        state["last_status_message"] = (
            "Session started from trusted home reference."
            if trusted
            else "Recovery required before motion."
        )
        self.save(state)
        return trusted

    def record_feedback(self, feedback_cm):
        state = self.load()
        state["last_feedback_cm"] = self._vector_or_none(feedback_cm)
        self.save(state)

    def record_command(self, command_cm):
        state = self.load()
        state["last_command_cm"] = self._vector_or_none(command_cm)
        self.save(state)

    def mark_shutdown_in_progress(self):
        state = self.load()
        state["shutdown_in_progress"] = True
        state["clean_shutdown"] = False
        state["last_status_message"] = "Clean shutdown in progress."
        self.save(state)

    def mark_clean_shutdown(self, feedback_cm=None):
        state = self.load()
        state["position_known"] = True
        state["home_confirmed"] = True
        state["recovery_required"] = False
        state["clean_shutdown"] = True
        state["shutdown_in_progress"] = False
        state["session_dirty"] = False
        state["owner_pid"] = None
        if feedback_cm is not None:
            state["last_feedback_cm"] = self._vector_or_none(feedback_cm)
        state["last_status_message"] = "Platform parked at home. Startup is trusted."
        self.save(state)

    def mark_unclean_shutdown(self, reason):
        state = self.load()
        state["position_known"] = False
        state["home_confirmed"] = False
        state["recovery_required"] = True
        state["clean_shutdown"] = False
        state["shutdown_in_progress"] = False
        state["session_dirty"] = False
        state["owner_pid"] = None
        state["last_status_message"] = reason
        self.save(state)

    def mark_manual_recovery_complete(self):
        state = self.load()
        state["position_known"] = True
        state["home_confirmed"] = True
        state["recovery_required"] = False
        state["clean_shutdown"] = True
        state["shutdown_in_progress"] = False
        state["session_dirty"] = False
        state["owner_pid"] = None
        state["last_status_message"] = (
            "Home reference manually confirmed. Motion is enabled."
        )
        self.save(state)

    def requires_recovery(self):
        state = self.load()
        return bool(state.get("recovery_required", True))

    def status_message(self):
        return self.load().get("last_status_message", "")
