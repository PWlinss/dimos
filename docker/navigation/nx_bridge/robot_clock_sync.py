#!/usr/bin/env python3
"""ROS-free robot-to-host wall-clock calibration for the NX bridge."""

import json
import math
import os
import tempfile
import time
import uuid
from collections import deque


_MAX_NANOSEC = 999_999_999
_DEFAULT_TTL_SEC = 2.0
_DEFAULT_MAX_AGE_SEC = 0.5
_DEFAULT_MAX_FUTURE_SEC = 0.05


def _read_boot_id():
    try:
        with open("/proc/sys/kernel/random/boot_id", encoding="ascii") as stream:
            return stream.read().strip()
    except OSError:
        return None


def stamp_seconds(sec, nanosec):
    if isinstance(sec, bool) or isinstance(nanosec, bool):
        raise ValueError("stamp fields must be integers")
    if not isinstance(sec, int) or not isinstance(nanosec, int):
        raise ValueError("stamp fields must be integers")
    if sec <= 0 or not 0 <= nanosec <= _MAX_NANOSEC:
        raise ValueError("invalid source stamp")
    result = sec + nanosec / 1_000_000_000
    if not math.isfinite(result):
        raise ValueError("non-finite source stamp")
    return result


def atomic_write_snapshot(path, snapshot):
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".robot-clock-", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(snapshot, stream, separators=(",", ":"))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def invalidate_snapshot(path):
    if path:
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass


def validate_positive_finite(value, name):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite positive number") from error
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be a finite positive number")
    return result


def read_snapshot(path, *, now_wall=None, now_mono=None, ttl_sec=_DEFAULT_TTL_SEC,
                  boot_id=None):
    if not path:
        raise ValueError("clock snapshot path is not configured")
    with open(path, encoding="utf-8") as stream:
        snapshot = json.load(stream)
    required = (
        "session", "boot_id", "offset_sec", "written_mono_ns", "anchor_wall_ns", "anchor_mono_ns"
    )
    if any(key not in snapshot for key in required):
        raise ValueError("incomplete clock snapshot")
    current_boot_id = _read_boot_id() if boot_id is None else boot_id
    if snapshot["boot_id"] != current_boot_id:
        raise ValueError("clock snapshot belongs to a different host boot")
    offset = float(snapshot["offset_sec"])
    if not math.isfinite(offset):
        raise ValueError("invalid clock offset")
    now_wall = time.time() if now_wall is None else now_wall
    now_mono = time.monotonic() if now_mono is None else now_mono
    written = int(snapshot["written_mono_ns"]) / 1e9
    age = now_mono - written
    if not 0 <= age <= ttl_sec:
        raise ValueError("clock snapshot is stale or from the future")
    anchor_wall = int(snapshot["anchor_wall_ns"]) / 1e9
    anchor_mono = int(snapshot["anchor_mono_ns"]) / 1e9
    if abs((now_wall - anchor_wall) - (now_mono - anchor_mono)) > 0.25:
        raise ValueError("NX wall clock jumped")
    return snapshot


def map_stamp(sec, nanosec, snapshot, *, now_wall=None, max_age_sec=_DEFAULT_MAX_AGE_SEC,
              max_future_sec=_DEFAULT_MAX_FUTURE_SEC):
    max_age_sec = validate_positive_finite(max_age_sec, "max_age_sec")
    max_future_sec = validate_positive_finite(max_future_sec, "max_future_sec")
    mapped = stamp_seconds(sec, nanosec) + float(snapshot["offset_sec"])
    if not math.isfinite(mapped):
        raise ValueError("invalid mapped stamp")
    now_wall = time.time() if now_wall is None else now_wall
    age = now_wall - mapped
    if age < -max_future_sec:
        raise ValueError("mapped message is from the future")
    if age > max_age_sec:
        raise ValueError("mapped message is stale")
    mapped_ns = round(mapped * 1_000_000_000)
    return divmod(mapped_ns, 1_000_000_000)


class SourceStampGuard:
    """Reject duplicate or reordered source stamps within one calibration session."""

    def __init__(self):
        self.session = None
        self.last_stamp_ns = None
        self.last_mapped_ns = None

    def accept(self, sec, nanosec, session, mapped_sec=None, mapped_nanosec=None):
        stamp_ns = sec * 1_000_000_000 + nanosec
        stamp_seconds(sec, nanosec)
        mapped_ns = None
        if mapped_sec is not None or mapped_nanosec is not None:
            if mapped_sec is None or mapped_nanosec is None:
                raise ValueError("mapped stamp requires both fields")
            mapped_ns = mapped_sec * 1_000_000_000 + mapped_nanosec
            stamp_seconds(mapped_sec, mapped_nanosec)
        if session != self.session:
            self.session = session
            self.last_stamp_ns = None
            self.last_mapped_ns = None
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            raise ValueError("source stamp is duplicate or reordered")
        if mapped_ns is not None and self.last_mapped_ns is not None and mapped_ns <= self.last_mapped_ns:
            raise ValueError("mapped stamp is duplicate or reordered")
        self.last_stamp_ns = stamp_ns
        self.last_mapped_ns = mapped_ns


class ClockCalibrator:
    """Estimate NX wall minus robot time using a low-latency sample window."""

    def __init__(self, path, *, warmup_samples=20, window_size=256, warmup_span_sec=0.5,
                 stable_spread_sec=0.5, jump_sec=0.25, write_interval_sec=0.2,
                 boot_id=None):
        self.path = path
        self.warmup_samples = warmup_samples
        self.samples = deque(maxlen=window_size)
        self.sample_mono_ns = deque(maxlen=window_size)
        self.warmup_span_sec = warmup_span_sec
        self.stable_spread_sec = stable_spread_sec
        self.jump_sec = jump_sec
        self.write_interval_sec = write_interval_sec
        self.boot_id = _read_boot_id() if boot_id is None else boot_id
        self.session = str(uuid.uuid4())
        self._last_robot = None
        self._last_wall_ns = None
        self._last_mono_ns = None
        self._last_write = 0.0
        self._valid_snapshot = False
        invalidate_snapshot(path)

    def observe(self, sec, nanosec, *, wall_ns=None, mono_ns=None):
        robot = stamp_seconds(sec, nanosec)
        wall_ns = time.time_ns() if wall_ns is None else int(wall_ns)
        mono_ns = time.monotonic_ns() if mono_ns is None else int(mono_ns)
        if self._last_robot is not None:
            elapsed_wall = (wall_ns - self._last_wall_ns) / 1e9
            elapsed_mono = (mono_ns - self._last_mono_ns) / 1e9
            robot_delta = robot - self._last_robot
            if elapsed_mono <= 0 or abs(elapsed_wall - elapsed_mono) > self.jump_sec:
                self.samples.clear()
                self.sample_mono_ns.clear()
                self.session = str(uuid.uuid4())
                invalidate_snapshot(self.path)
                self._valid_snapshot = False
            elif robot_delta <= 0 and abs(robot_delta - elapsed_mono) <= self.jump_sec:
                # A duplicate or nearby reordered packet is noise; keep last accepted anchors.
                return None
            elif abs(robot_delta - elapsed_mono) > self.jump_sec:
                self.samples.clear()
                self.sample_mono_ns.clear()
                self.session = str(uuid.uuid4())
                invalidate_snapshot(self.path)
                self._valid_snapshot = False
        self._last_robot = robot
        self._last_wall_ns = wall_ns
        self._last_mono_ns = mono_ns
        self.samples.append(wall_ns / 1e9 - robot)
        self.sample_mono_ns.append(mono_ns)
        if len(self.samples) < self.warmup_samples:
            return None
        observed_span = (self.sample_mono_ns[-1] - self.sample_mono_ns[0]) / 1e9
        if observed_span < self.warmup_span_sec:
            return None
        ordered = sorted(self.samples)
        # Low-tail offset rejects most receive-side transport/scheduling latency.
        offset = ordered[int((len(ordered) - 1) * 0.1)]
        if ordered[-1] - offset > self.stable_spread_sec:
            invalidate_snapshot(self.path)
            if self._valid_snapshot:
                self.session = str(uuid.uuid4())
            self._valid_snapshot = False
            self.samples.clear()
            self.sample_mono_ns.clear()
            self.samples.append(wall_ns / 1e9 - robot)
            self.sample_mono_ns.append(mono_ns)
            return None
        if mono_ns / 1e9 - self._last_write < self.write_interval_sec:
            return offset
        snapshot = {
            "session": self.session,
            "boot_id": self.boot_id,
            "offset_sec": offset,
            "written_mono_ns": mono_ns,
            "anchor_wall_ns": wall_ns,
            "anchor_mono_ns": mono_ns,
        }
        atomic_write_snapshot(self.path, snapshot)
        self._last_write = mono_ns / 1e9
        self._valid_snapshot = True
        return offset
