#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fail-closed Gazebo Sim 8 DetachableJoint control for the DrinkTask chassis."""
import subprocess
import threading
import time

try:
    from gz.msgs10.stringmsg_pb2 import StringMsg
except Exception:  # pragma: no cover - exercised in simulator runtime
    StringMsg = None


class GazeboGroundLock:
    """Control and verify the physical world-anchor joint independently of cmd_mux."""

    def __init__(self, transport_node, world_name="follow_world"):
        if StringMsg is None:
            raise RuntimeError("gz.msgs10 StringMsg bindings are unavailable")
        self.attach_topic = f"/world/{world_name}/chassis_lock/attach"
        self.detach_topic = f"/world/{world_name}/chassis_lock/detach"
        self.state_topic = f"/world/{world_name}/chassis_lock/state"
        self._mutex = threading.Lock()
        self._state = None
        self._state_time = 0.0
        self.last_command_attempts = 0
        self.last_command_error = ""
        if not transport_node.subscribe(StringMsg, self.state_topic, self._on_state):
            raise RuntimeError("cannot subscribe to " + self.state_topic)

    def _on_state(self, message):
        state = str(message.data).strip().lower()
        if state not in ("attached", "detached"):
            return
        with self._mutex:
            self._state = state
            self._state_time = time.monotonic()

    @property
    def state(self):
        with self._mutex:
            return self._state

    @property
    def state_age_s(self):
        with self._mutex:
            return (time.monotonic() - self._state_time
                    if self._state_time else float("inf"))

    def wait_for_state(self, expected, timeout_s=5.0, spin=None,
                       after_time=None):
        """Wait for a transition newer than command start; states are event-driven."""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            with self._mutex:
                state = self._state
                state_time = self._state_time
            if state == expected and (
                    after_time is None or state_time >= after_time):
                return True
            if spin is None:
                time.sleep(0.02)
            else:
                spin(0.02)
        with self._mutex:
            return (self._state == expected and
                    (after_time is None or self._state_time >= after_time))

    def command(self, expected, timeout_s=5.0, spin=None, force=False):
        if expected not in ("attached", "detached"):
            raise ValueError("expected state must be attached or detached")
        if self.state == expected and not force:
            return True
        topic = self.attach_topic if expected == "attached" else self.detach_topic
        # A short-lived one-shot publisher can exit before Gazebo Transport
        # discovery delivers its message. Retry idempotent requests throughout
        # the acknowledgement window, matching the proven ground-lock probe.
        # Every success still requires a fresh state event newer than a request.
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        self.last_command_attempts = 0
        self.last_command_error = ""
        while time.monotonic() < deadline:
            command_started = time.monotonic()
            remaining = max(0.0, deadline - command_started)
            try:
                result = subprocess.run(
                    ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", " "],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, timeout=min(2.0, max(0.2, remaining)),
                    check=False)
                self.last_command_attempts += 1
                if result.returncode != 0:
                    self.last_command_error = (
                        "publisher exit %d: %s" % (result.returncode, result.stdout))
                elif self.wait_for_state(
                        expected, timeout_s=min(0.25, remaining), spin=spin,
                        after_time=command_started):
                    return True
            except subprocess.TimeoutExpired as exc:
                self.last_command_attempts += 1
                self.last_command_error = "publisher timeout: %s" % exc
            except OSError as exc:
                self.last_command_error = "publisher unavailable: %s" % exc
                return False

            if spin is None:
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return False
