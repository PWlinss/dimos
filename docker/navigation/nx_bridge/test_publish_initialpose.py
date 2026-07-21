import json
import math
import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).with_name("publish_initialpose.py")


def test_dry_run_prints_initialpose_payload() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--x",
            "1.25",
            "--y",
            "-2.5",
            "--yaw",
            "1.57079632679",
            "--dry-run",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(result.stdout)

    assert payload["frame_id"] == "map"
    assert payload["x"] == 1.25
    assert payload["y"] == -2.5
    assert payload["yaw"] == 1.57079632679
    assert payload["orientation"]["z"] == pytest_approx(math.sin(1.57079632679 / 2.0))
    assert payload["orientation"]["w"] == pytest_approx(math.cos(1.57079632679 / 2.0))


def test_initialpose_defaults_to_one_publish_to_avoid_reinitialization_storms() -> None:
    source = SCRIPT.read_text()

    assert 'parser.add_argument("--count", type=int, default=1' in source


def pytest_approx(value: float):
    import pytest

    return pytest.approx(value)
