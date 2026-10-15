from threading import RLock
from types import SimpleNamespace

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage

from dimos.agents.agent import Agent
from dimos.perception.spatial_perception import SpatialMemory
from dimos.web.websocket_vis.agent_memory import (
    NXAgentMemoryWebModule,
    serialize_chat_message,
)
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def test_chat_serialization_and_bounded_reconnect_history() -> None:
    human = serialize_chat_message(HumanMessage(content="hello", id="human-1"))
    ai = serialize_chat_message(
        AIMessage(
            content="",
            id="ai-1",
            tool_calls=[{"name": "navigate", "args": {"room": "kitchen"}, "id": "call-1"}],
        )
    )
    tool = serialize_chat_message(
        ToolMessage(content="started", tool_call_id="call-1", name="navigate")
    )

    assert human["role"] == "human" and human["id"] == "human-1"
    assert ai["role"] == "ai" and ai["tool_calls"][0]["args"] == {"room": "kitchen"}
    assert tool["role"] == "tool" and tool["tool_call_id"] == "call-1"

    events: list[tuple[str, object]] = []
    fake = SimpleNamespace(
        _chat_messages=[],
        state_lock=RLock(),
        _emit=lambda event, data: events.append((event, data)),
    )
    for index in range(205):
        NXAgentMemoryWebModule._on_agent_message(
            fake, HumanMessage(content=str(index), id=f"message-{index}")
        )
    fake._agent_available = lambda: True
    fake._agent_idle = False
    fake._memory = object()
    state = NXAgentMemoryWebModule._connection_state(fake)

    assert len(state["agent_messages"]) == 200
    assert state["agent_messages"][0]["id"] == "message-5"
    assert state["agent_messages"][-1]["id"] == "message-204"
    assert state["agent_messages"][-1]["id"] == events[-1][1]["id"]
    assert state["agent_available"] is True and state["agent_idle"] is False
    assert state["memory_available"] is True


def test_agent_send_validates_and_publishes_human_input_only() -> None:
    published: list[str] = []
    fake = SimpleNamespace(
        human_input=SimpleNamespace(transport=object(), publish=published.append),
        agent=SimpleNamespace(transport=object()),
        state_lock=RLock(),
        _agent_idle=True,
    )
    fake._agent_available = lambda: (
        fake.agent.transport is not None and fake.human_input.transport is not None
    )
    fake._emit = lambda *_args: None

    assert _run(NXAgentMemoryWebModule._on_agent_send(fake, "sid", {"text": " go "})) == {
        "ok": True
    }
    assert published == [" go "]
    assert _run(NXAgentMemoryWebModule._on_agent_send(fake, "sid", {"text": "  "}))[
        "ok"
    ] is False
    assert _run(
        NXAgentMemoryWebModule._on_agent_send(fake, "sid", {"text": "x" * 4001})
    )["ok"] is False

    fake.human_input.transport = None
    result = _run(NXAgentMemoryWebModule._on_agent_send(fake, "sid", {"text": "hello"}))
    assert result == {"ok": False, "error": "agent transport is not bound"}
    assert published == [" go "]
    fake.human_input.transport = object()
    fake._agent_idle = False
    assert _run(NXAgentMemoryWebModule._on_agent_send(fake, "sid", {"text": "busy"})) == {
        "ok": False,
        "error": "agent is not idle",
    }
    assert published == [" go "]


def test_viewer_none_still_starts_command_center_web_server(monkeypatch) -> None:
    started: list[bool] = []
    monkeypatch.setattr(WebsocketVisModule, "start", lambda _self: started.append(True))
    subscriptions: list[object] = []
    stream = SimpleNamespace(subscribe=lambda callback: subscriptions.append(callback))
    fake = object.__new__(NXAgentMemoryWebModule)
    fake.__dict__.update(
        _global_config=SimpleNamespace(viewer="none"),
        _disposables=SimpleNamespace(add=lambda _disposable: None),
        agent=stream,
        agent_idle=stream,
        color_image=stream,
    )

    NXAgentMemoryWebModule.start(fake)

    assert started == [True]
    assert len(subscriptions) == 3


def test_camera_stream_emits_at_most_five_frames_per_second(monkeypatch) -> None:
    events: list[tuple[str, object]] = []
    times = iter((1.0, 1.1, 1.21))
    monkeypatch.setattr("dimos.web.websocket_vis.agent_memory.time.monotonic", lambda: next(times))
    fake = SimpleNamespace(
        state_lock=RLock(),
        vis_state={},
        _last_camera_emit=0.0,
        _emit=lambda event, frame: events.append((event, frame)),
    )

    class Frame:
        ts = 123.0

        def to_base64(self, **kwargs):  # type: ignore[no-untyped-def]
            assert kwargs == {"quality": 70, "max_width": 960, "max_height": 540}
            return "jpeg-data"

    for _ in range(3):
        NXAgentMemoryWebModule._on_color_image(fake, Frame())  # type: ignore[arg-type]

    assert len(events) == 2
    assert events[0] == (
        "camera_frame",
        {"src": "data:image/jpeg;base64,jpeg-data", "ts": 123.0},
    )
    assert fake.vis_state["camera_frame"] == events[-1][1]


def test_memory_search_rejects_queries_over_512_characters() -> None:
    fake = SimpleNamespace()
    result = _run(
        NXAgentMemoryWebModule._on_memory_search(
            fake, "sid", {"query": "x" * 513}
        )
    )

    assert result == {"ok": False, "error": "query must be at most 512 characters"}


def test_agent_publishes_initial_idle_after_graph_and_worker_are_ready(monkeypatch) -> None:
    class Worker:
        started = False

        def start(self):
            self.started = True

    idle_values: list[bool] = []
    worker = Worker()
    fake = SimpleNamespace(
        rpc=object(),
        config=SimpleNamespace(
            model_fixture=None, model_kwargs={}, model="test", system_prompt="test prompt"
        ),
        _lock=RLock(),
        _state_graph=None,
        _thread=worker,
        agent_idle=SimpleNamespace(publish=idle_values.append),
    )
    monkeypatch.setattr("langchain.agents.create_agent", lambda **_kwargs: object())

    Agent.on_system_modules(fake, [])

    assert fake._state_graph is not None
    assert worker.started is True
    assert idle_values == [True]


def test_spatial_memory_visual_apis_are_bounded_read_only_and_locked() -> None:
    class Lock:
        active = False

        def __enter__(self):
            self.active = True

        def __exit__(self, *_args):
            self.active = False

    lock = Lock()

    class Collection:
        def get(self, *, include):
            assert lock.active
            if include == ["metadatas"]:
                return {
                "ids": ["old", "new"],
                "metadatas": [
                    {"timestamp": 1, "pos_x": 1, "coordinate_frame": "odom"},
                    {"timestamp": 2, "pos_x": 2, "coordinate_frame": "odom"},
                ],
                }
            assert include == ["metadatas", "documents"]
            return {"ids": [], "metadatas": [], "documents": []}

    class VectorDB:
        image_collection = Collection()
        location_collection = Collection()
        queried: list[tuple[str, int]] = []

        def query_by_text(self, query, limit):
            assert lock.active
            self.queried.append((query, limit))
            return [
                {"id": f"result-{i}", "metadata": {"timestamp": i}, "distance": 0.1 * i}
                for i in range(5)
            ]

    memory = SpatialMemory.__new__(SpatialMemory)
    memory._process_lock = lock
    memory.vector_db = VectorDB()
    memory._visual_memory = SimpleNamespace(images={})
    memory.world_frame_id = "odom"
    memory.frame_count = 4
    memory.stored_frame_count = 2
    memory.robot_locations = []

    snapshot = memory.get_visualization_snapshot()
    results = memory.search_visualization("blue sofa")

    assert snapshot["stats"]["stored_frame_count"] == 2
    assert [record["id"] for record in snapshot["records"]] == ["new", "old"]
    assert all(record["thumbnail"] is None for record in snapshot["records"])
    assert len(results) == 3 and results[0]["similarity"] == 1.0
    assert memory.vector_db.queried == [("blue sofa", 3)]
    assert lock.active is False


def _run(coro):
    import asyncio

    return asyncio.run(coro)
