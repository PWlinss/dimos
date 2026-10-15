import ast
import importlib.util
import json
import queue
import struct
import time
from types import SimpleNamespace
from pathlib import Path

import pytest


SOURCE_PATH = Path(__file__).with_name("robot_clock_sync.py")
SPEC = importlib.util.spec_from_file_location("robot_clock_sync", SOURCE_PATH)
clock = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(clock)


def _seed(calibrator, *, base_robot=1_800_000_000.0, base_wall=1_800_000_208.1,
          base_mono=100.0):
    latencies = (0.010, 0.008, 0.060, 0.012, 0.009, 0.011)
    for index in range(31):
        latency = latencies[index % len(latencies)]
        robot = base_robot + index * 0.02
        mono = base_mono + index * 0.02
        wall = base_wall + index * 0.02 + latency
        sec, nanosec = divmod(round(robot * 1e9), 1_000_000_000)
        calibrator.observe(sec, nanosec, wall_ns=round(wall * 1e9), mono_ns=round(mono * 1e9))


def test_one_snapshot_maps_fast_odom_and_60ms_cloud_latency(tmp_path):
    path = tmp_path / "clock.json"
    calibrator = clock.ClockCalibrator(str(path))
    _seed(calibrator)
    snapshot = clock.read_snapshot(str(path), now_wall=1_800_000_208.72, now_mono=100.62)

    # Both message types use the same Sport-derived offset; packet delay is not added to stamps.
    robot_stamp = 1_800_000_000.59
    sec, nanosec = divmod(round(robot_stamp * 1e9), 1_000_000_000)
    mapped = clock.map_stamp(sec, nanosec, snapshot, now_wall=1_800_000_208.72)
    assert mapped == pytest.approx((1_800_000_208, 698_000_000), abs=1_000)
    cloud_sec, cloud_nanosec = divmod(round(1_800_000_000.54 * 1e9), 1_000_000_000)
    cloud_mapped = clock.map_stamp(
        cloud_sec, cloud_nanosec, snapshot, now_wall=1_800_000_208.708
    )
    assert cloud_mapped == pytest.approx((1_800_000_208, 648_000_000), abs=1_000)
    assert snapshot["offset_sec"] == pytest.approx(208.108, abs=0.002)


def test_jitter_is_bounded_and_bad_or_stale_snapshots_drop(tmp_path):
    path = tmp_path / "clock.json"
    calibrator = clock.ClockCalibrator(str(path))
    _seed(calibrator)
    snapshot = clock.read_snapshot(str(path), now_wall=1_800_000_208.72, now_mono=100.62)
    sec, nanosec = divmod(round(1_800_000_000.59 * 1e9), 1_000_000_000)
    assert clock.map_stamp(sec, nanosec, snapshot, now_wall=1_800_000_208.72)[0] > 0
    with pytest.raises(ValueError, match="stale"):
        clock.read_snapshot(str(path), now_wall=1_800_000_210.7, now_mono=102.6)
    with pytest.raises(ValueError, match="future"):
        clock.map_stamp(sec, nanosec, snapshot, now_wall=1_800_000_207.0)
    with pytest.raises(ValueError, match="invalid source stamp"):
        clock.map_stamp(sec, 1_000_000_000, snapshot, now_wall=1_800_000_208.72)
    with pytest.raises(FileNotFoundError):
        clock.read_snapshot(str(tmp_path / "missing.json"))


@pytest.mark.parametrize("value", (0, -1, float("nan"), float("inf"), -float("inf"), "bad"))
def test_max_source_age_must_be_finite_and_positive(value):
    with pytest.raises(ValueError, match="finite positive"):
        clock.validate_positive_finite(value, "max_source_age_sec")


def test_calibration_waits_for_minimum_observed_span_and_invalidates_on_instability(tmp_path):
    path = tmp_path / "clock.json"
    calibrator = clock.ClockCalibrator(str(path), warmup_span_sec=0.8)
    _seed(calibrator)
    assert not path.exists()

    for index in range(31, 42):
        robot = 1_800_000_000 + index * 0.02
        sec, nanosec = divmod(round(robot * 1e9), 1_000_000_000)
        mono = 100 + index * 0.02
        wall = 1_800_000_208.1 + index * 0.02 + 0.01
        calibrator.observe(sec, nanosec, wall_ns=round(wall * 1e9), mono_ns=round(mono * 1e9))
    assert path.exists()

    for index in range(42, 70):
        robot = 1_800_000_000 + index * 0.02
        sec, nanosec = divmod(round(robot * 1e9), 1_000_000_000)
        mono = 100 + index * 0.02
        wall = 1_800_000_208.1 + index * 0.02 + 0.01 + (index - 41) * 0.03
        calibrator.observe(sec, nanosec, wall_ns=round(wall * 1e9), mono_ns=round(mono * 1e9))
    assert not path.exists()
    assert not calibrator._valid_snapshot


def _calibrator_with_snapshot(path):
    calibrator = clock.ClockCalibrator(
        str(path), warmup_samples=2, warmup_span_sec=0.0, write_interval_sec=0.0
    )
    calibrator.observe(
        1_800_000_000, 0, wall_ns=1_800_000_208_000_000_000, mono_ns=100_000_000_000
    )
    calibrator.observe(
        1_800_000_000, 10_000_000,
        wall_ns=1_800_000_208_010_000_000,
        mono_ns=100_010_000_000,
    )
    return calibrator


def test_duplicate_and_small_reorder_preserve_calibration_anchors(tmp_path):
    path = tmp_path / "clock.json"
    calibrator = _calibrator_with_snapshot(path)
    session = calibrator.session
    anchors = (calibrator._last_robot, calibrator._last_wall_ns, calibrator._last_mono_ns)
    for source_ns, elapsed_ns in ((10_000_000, 626_000), (9_500_000, 1_192_000)):
        calibrator.observe(
            1_800_000_000,
            source_ns,
            wall_ns=anchors[1] + elapsed_ns,
            mono_ns=anchors[2] + elapsed_ns,
        )
        assert path.exists()
        assert calibrator.session == session
        assert (calibrator._last_robot, calibrator._last_wall_ns, calibrator._last_mono_ns) == anchors


def test_sustained_freeze_eventually_invalidates_using_last_accepted_anchor(tmp_path):
    path = tmp_path / "clock.json"
    calibrator = _calibrator_with_snapshot(path)
    anchors = (calibrator._last_wall_ns, calibrator._last_mono_ns)
    session = calibrator.session
    calibrator.observe(
        1_800_000_000, 10_000_000,
        wall_ns=anchors[0] + 100_000_000,
        mono_ns=anchors[1] + 100_000_000,
    )
    assert path.exists()
    calibrator.observe(
        1_800_000_000, 10_000_000,
        wall_ns=anchors[0] + 251_000_000,
        mono_ns=anchors[1] + 251_000_000,
    )
    assert not path.exists()
    assert calibrator.session != session


def test_wall_jump_on_duplicate_and_major_source_jump_still_reset(tmp_path):
    for jump_wall, jump_source in ((True, False), (False, True)):
        path = tmp_path / f"clock-{jump_wall}-{jump_source}.json"
        calibrator = _calibrator_with_snapshot(path)
        session = calibrator.session
        mono_ns = calibrator._last_mono_ns + 10_000_000
        wall_ns = calibrator._last_wall_ns + (1_000_000_000 if jump_wall else 10_000_000)
        sec, nanosec = (1_800_000_000, 10_000_000)
        if jump_source:
            sec, nanosec = 1_800_000_001, 20_000_000
            wall_ns = calibrator._last_wall_ns + 10_000_000
        calibrator.observe(sec, nanosec, wall_ns=wall_ns, mono_ns=mono_ns)
        assert not path.exists()
        assert calibrator.session != session


def test_source_regression_and_robot_clock_jump_rewarm_new_session(tmp_path):
    path = tmp_path / "clock.json"
    calibrator = clock.ClockCalibrator(str(path))
    _seed(calibrator)
    old_session = calibrator.session
    assert path.exists()
    sec, nanosec = divmod(round(1_800_000_000.08 * 1e9), 1_000_000_000)
    calibrator.observe(sec, nanosec, wall_ns=1_800_000_208_73e7, mono_ns=100_620_000_000)
    assert calibrator.session != old_session
    assert len(calibrator.samples) == 1
    assert not path.exists()

    calibrator.observe(sec + 1, nanosec, wall_ns=1_800_000_208_75e7, mono_ns=100_640_000_000)
    assert len(calibrator.samples) == 1


def test_nx_wall_clock_jump_invalidates_snapshot_read(tmp_path):
    path = tmp_path / "clock.json"
    calibrator = clock.ClockCalibrator(str(path))
    _seed(calibrator)
    with pytest.raises(ValueError, match="wall clock jumped"):
        clock.read_snapshot(str(path), now_wall=1_800_000_209.0, now_mono=100.62)


def test_snapshot_from_previous_host_boot_is_rejected(tmp_path):
    path = tmp_path / "clock.json"
    calibrator = clock.ClockCalibrator(str(path), boot_id="boot-a")
    _seed(calibrator)
    with pytest.raises(ValueError, match="different host boot"):
        clock.read_snapshot(str(path), now_wall=1_800_000_208.72, now_mono=100.62,
                            boot_id="boot-b")


def test_startup_invalidation_session_and_frame_data_contract(tmp_path):
    path = tmp_path / "clock.json"
    path.write_text(json.dumps({"session": "old"}), encoding="utf-8")
    calibrator = clock.ClockCalibrator(str(path))
    assert not path.exists()
    assert calibrator.session != "old"

    source = Path(__file__).with_name("pointcloud_tcp_bridge.py").read_text(encoding="utf-8")
    assert "message.header.stamp.sec = mapped_sec" in source
    assert "message.header.stamp.nanosec = mapped_nanosec" in source
    assert "message.data =" not in source
    assert "message.header.frame_id =" not in source


def test_duplicate_and_reordered_sources_are_rejected_and_reboot_resets_guard():
    guard = clock.SourceStampGuard()
    guard.accept(100, 20, "session-a")
    with pytest.raises(ValueError, match="duplicate or reordered"):
        guard.accept(100, 20, "session-a")
    with pytest.raises(ValueError, match="duplicate or reordered"):
        guard.accept(100, 10, "session-a")
    guard.accept(1, 0, "session-b")
    with pytest.raises(ValueError, match="duplicate or reordered"):
        guard.accept(1, 0, "session-b")


def test_rolling_offset_cannot_make_mapped_stamp_go_backwards():
    guard = clock.SourceStampGuard()
    source1 = divmod(round(1_800_000_000.59 * 1e9), 1_000_000_000)
    source2 = divmod(round(1_800_000_000.60 * 1e9), 1_000_000_000)
    mapped1 = clock.map_stamp(*source1, {"offset_sec": 208.108}, now_wall=1_800_000_208.72)
    mapped2 = clock.map_stamp(*source2, {"offset_sec": 208.09}, now_wall=1_800_000_208.72)
    guard.accept(*source1, "session", *mapped1)
    with pytest.raises(ValueError, match="mapped stamp is duplicate or reordered"):
        guard.accept(*source2, "session", *mapped2)


def _extract_method(source_path, class_name, method_name, namespace):
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    owner = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
    method = next(node for node in owner.body if isinstance(node, ast.FunctionDef) and node.name == method_name)
    module = ast.Module(body=[method], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(source_path), "exec"), namespace)
    return namespace[method_name]


def test_strict_udp_and_cloud_consumers_drop_mapped_time_regression():
    now_ns = round(time.time() * 1e9)
    source1 = divmod(now_ns - 150_000_000, 1_000_000_000)
    source2 = divmod(now_ns - 140_000_000, 1_000_000_000)
    snapshots = iter(({"session": "current", "offset_sec": 0.05},
                      {"session": "current", "offset_sec": 0.0}))

    class FakeSocket:
        def __init__(self, packets):
            self.packets = iter(packets)

        def recvfrom(self, _):
            try:
                return next(self.packets), None
            except StopIteration as error:
                raise BlockingIOError from error

    class FakePose:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.pose = SimpleNamespace(
                position=SimpleNamespace(), orientation=SimpleNamespace()
            )

    pose_packets = [
        json.dumps({"source_stamp": {"sec": sec, "nanosec": ns},
                    "clock_session": "current", "position": [1, 2, 3],
                    "quaternion": [1, 0, 0, 0]}).encode()
        for sec, ns in (source1, source2)
    ]
    poses = []
    udp_ns = {"json": json, "PoseStamped": FakePose,
              "Time": lambda sec, nanosec: SimpleNamespace(sec=sec, nanosec=nanosec),
              "read_snapshot": lambda _: next(snapshots), "map_stamp": clock.map_stamp,
              "time": time}
    udp_method = _extract_method(
        SOURCE_PATH.with_name("udp_to_pose.py"), "UdpToPose", "_drain_packets", udp_ns
    )
    udp = SimpleNamespace(
        socket=FakeSocket(pose_packets), timestamp_mode="robot_clock", frame_id="odom",
        clock_sync_file="clock", max_source_age_sec=0.5,
        source_stamp_guard=clock.SourceStampGuard(), publisher=SimpleNamespace(publish=poses.append),
        _warn_drop=lambda _: None,
    )
    udp_method(udp)
    assert len(poses) == 1

    cloud_snapshots = iter(({"session": "current", "offset_sec": 0.05},
                            {"session": "current", "offset_sec": 0.0}))
    cloud_payloads = []
    metadata = struct.Struct("!qIQQ36s")
    for sec, ns in (source1, source2):
        native = json.dumps({"sec": sec, "nanosec": ns}).encode()
        cloud_payloads.append(
            metadata.pack(sec, ns, now_ns, now_ns, b"current".ljust(36, b"\0")) + native
        )

    class OnePacketQueue:
        def __init__(self, payload):
            self.payload = payload

        def get_nowait(self):
            if self.payload is None:
                raise queue.Empty
            payload, self.payload = self.payload, None
            return payload

    clouds = []
    def deserialize_cloud(payload, _):
        stamp = json.loads(payload)
        return SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(**stamp), frame_id="odom"), data=b"xyz"
        )

    cloud_ns = {"queue": queue, "time": time, "_CLOUD_METADATA": metadata,
                "read_snapshot": lambda _: next(cloud_snapshots),
                "map_stamp": clock.map_stamp, "deserialize_message": deserialize_cloud,
                "PointCloud2": object}
    cloud_method = _extract_method(
        SOURCE_PATH.with_name("pointcloud_tcp_bridge.py"), "PointCloudTcpReceiver",
        "_publish_latest", cloud_ns
    )
    receiver = SimpleNamespace(
        _clock_sync_file="clock", _max_source_age_sec=0.5,
        _source_stamp_guard=clock.SourceStampGuard(), _receive_failures=0,
        _last_drop_log=0.0, _published=0, _publisher=SimpleNamespace(publish=clouds.append),
        get_logger=lambda: SimpleNamespace(warning=lambda _: None),
    )
    for payload in cloud_payloads:
        receiver._latest = OnePacketQueue(payload)
        cloud_method(receiver)
    assert len(clouds) == 1
    assert clouds[0].header.frame_id == "odom"
    assert clouds[0].data == b"xyz"
