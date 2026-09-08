import ast
import math
import socket
import struct
from pathlib import Path

import pytest


SOURCE_PATH = Path(__file__).with_name("pointcloud_tcp_bridge.py")
SOURCE = SOURCE_PATH.read_text(encoding="utf-8")


def _load_pure_helpers() -> dict:
    tree = ast.parse(SOURCE, filename=str(SOURCE_PATH))
    names = {
        "_recv_exact",
        "_retry_delay",
        "_validate_nonnegative_finite",
        "_validate_positive_finite",
        "_validate_positive_int",
    }
    body = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    module = ast.Module(body=body, type_ignores=[])
    namespace = {
        "math": math,
        "socket": socket,
        "struct": struct,
        "_MIN_RETRY_BACKOFF_SEC": 0.01,
    }
    exec(compile(ast.fix_missing_locations(module), str(SOURCE_PATH), "exec"), namespace)
    return namespace


def test_pointcloud_tcp_bridge_contract() -> None:
    ast.parse(SOURCE)
    assert 'choices=("sender", "receiver")' in SOURCE
    assert '"/utlidar/cloud_deskewed"' in SOURCE
    assert '"/dimos/registered_scan"' in SOURCE
    assert '"127.0.0.1"' in SOURCE
    assert "_DEFAULT_PORT = 39102" in SOURCE
    assert "_DEFAULT_MAX_MESSAGE_BYTES = 16 * 1024 * 1024" in SOURCE
    assert 'struct.Struct("!I")' in SOURCE
    assert "serialize_message(message)" in SOURCE
    assert "deserialize_message(payload, PointCloud2)" in SOURCE
    assert "queue.Queue(maxsize=1)" in SOURCE
    assert 'declare_parameter("input_reliability", "reliable")' in SOURCE
    assert 'declare_parameter("connect_timeout_sec", _DEFAULT_CONNECT_TIMEOUT_SEC)' in SOURCE
    assert "retry_backoff_initial_sec" in SOURCE
    assert "retry_backoff_max_sec" in SOURCE
    assert "failed to bind/listen" in SOURCE


def test_parameter_validation_rejects_non_positive_or_non_finite_values() -> None:
    helpers = _load_pure_helpers()
    positive_finite = helpers["_validate_positive_finite"]
    positive_int = helpers["_validate_positive_int"]

    assert positive_finite("0.25", "connect_timeout_sec") == pytest.approx(0.25)
    assert positive_int("4", "max_message_bytes") == 4

    for value in (0, -1, math.inf, -math.inf, math.nan, "not-a-number"):
        with pytest.raises(ValueError):
            positive_finite(value, "max_rate_hz")

    for value in (0, -1, 1.5, math.inf, math.nan, "not-an-integer"):
        with pytest.raises(ValueError):
            positive_int(value, "max_message_bytes")


def test_retry_delay_has_a_lower_bound() -> None:
    retry_delay = _load_pure_helpers()["_retry_delay"]

    assert retry_delay(0, "retry_backoff_initial_sec") == pytest.approx(0.01)
    assert retry_delay(0.25, "retry_backoff_initial_sec") == pytest.approx(0.25)


class _PartialReader:
    def __init__(self, data: bytes, chunk_size: int) -> None:
        self._data = data
        self._chunk_size = chunk_size

    def recv(self, size: int) -> bytes:
        if not self._data:
            return b""
        chunk_size = min(size, self._chunk_size, len(self._data))
        chunk, self._data = self._data[:chunk_size], self._data[chunk_size:]
        return chunk


def test_framing_handles_partial_reads() -> None:
    helpers = _load_pure_helpers()
    recv_exact = helpers["_recv_exact"]
    length = struct.Struct("!I")
    payload = b"pointcloud-payload"
    reader = _PartialReader(length.pack(len(payload)) + payload, chunk_size=2)

    payload_size = length.unpack(recv_exact(reader, length.size))[0]
    assert payload_size == len(payload)
    assert recv_exact(reader, payload_size) == payload

    with pytest.raises(ConnectionError):
        recv_exact(_PartialReader(b"\x00", chunk_size=1), length.size)
