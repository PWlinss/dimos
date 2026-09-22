from pathlib import Path


ROOT = Path(__file__).resolve().parent
START_NX_BRIDGE = (ROOT / "start_nx_bridge.sh").read_text(encoding="utf-8")
WIRELESS_CONTROL = (ROOT / "dimos-wireless-data-bridge-control.sh").read_text(encoding="utf-8")
PUSAI_START = (ROOT.parents[2] / "scripts" / "start_pusai_nx_bridge.sh").read_text(encoding="utf-8")
CAMERA_DROPIN = (
    ROOT / "systemd" / "go2-camera-source.service.d" / "20-camera-rate.conf"
).read_text(encoding="utf-8")


def test_camera_relays_default_to_five_fps() -> None:
    assert 'CAMERA_RELAY_MAX_RATE_HZ="${DIMOS_NX_CAMERA_RELAY_MAX_RATE_HZ:-5.0}"' in START_NX_BRIDGE
    assert 'camera_relay_max_rate_hz="${DIMOS_NX_CAMERA_RELAY_MAX_RATE_HZ:-5.0}"' in WIRELESS_CONTROL
    assert '-p max_rate_hz:="${camera_relay_max_rate_hz}"' in WIRELESS_CONTROL


def test_pusai_exports_separate_rerun_rates() -> None:
    assert 'export DIMOS_NX_RERUN_IMAGE_INTERVAL="${DIMOS_NX_RERUN_IMAGE_INTERVAL:-0.2}"' in PUSAI_START
    assert 'export DIMOS_NX_RERUN_POINTCLOUD_INTERVAL="${DIMOS_NX_RERUN_POINTCLOUD_INTERVAL:-5.0}"' in PUSAI_START


def test_go2_systemd_dropin_replaces_execstart_at_five_fps() -> None:
    assert "ExecStart=\n" in CAMERA_DROPIN
    assert "target_fps:=5.0" in CAMERA_DROPIN
    assert "target_fps:=2.0" not in CAMERA_DROPIN
