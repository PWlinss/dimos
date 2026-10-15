# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""WebSocket chat and read-only spatial-memory views for the NX panel."""

import asyncio
from collections.abc import Callable
import time
from typing import Any, Literal, Protocol, TypedDict
import uuid

from langchain_core.messages.base import BaseMessage
from reactivex.disposable import Disposable
from typing_extensions import NotRequired

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs import Image
from dimos.spec.utils import Spec
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


class Position(TypedDict):
    x: float
    y: float
    z: float


class MemoryRecord(TypedDict):
    id: str
    timestamp: float
    coordinate_frame: str
    position: Position
    thumbnail: str | None
    similarity: float | None
    label: str | None


class MemoryLocation(TypedDict):
    id: str
    name: str
    position: Position
    coordinate_frame: str
    timestamp: float


class MemorySnapshot(TypedDict):
    stats: dict[str, int]
    records: list[MemoryRecord]
    locations: list[MemoryLocation]


class SpatialVisualizationSpec(Spec, Protocol):
    def get_visualization_snapshot(self) -> MemorySnapshot: ...

    def search_visualization(self, query: str) -> list[MemoryRecord]: ...


class ChatMessage(TypedDict):
    id: str
    role: Literal["human", "ai", "tool"]
    content: str
    tool_calls: list[dict[str, Any]]
    ts: float
    tool_call_id: NotRequired[str]
    name: NotRequired[str]


class NXAgentMemoryWebModule(WebsocketVisModule):
    """Add agent chat and bounded spatial-memory reads to the existing panel."""

    agent: In[BaseMessage]
    agent_idle: In[bool]
    color_image: In[Image]
    human_input: Out[str]

    _memory: SpatialVisualizationSpec

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._chat_messages: list[ChatMessage] = []
        self._agent_idle: bool | None = None
        self._last_camera_emit = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.agent.subscribe(self._on_agent_message)))
        self._disposables.add(Disposable(self.agent_idle.subscribe(self._on_agent_idle)))
        self._disposables.add(Disposable(self.color_image.subscribe(self._on_color_image)))

    def _on_color_image(self, image: Image) -> None:
        now = time.monotonic()
        with self.state_lock:
            if now - self._last_camera_emit < 0.2:
                return
            self._last_camera_emit = now
            frame = {
                "src": f"data:image/jpeg;base64,{image.to_base64(quality=70, max_width=960, max_height=540)}",
                "ts": image.ts,
            }
            self.vis_state["camera_frame"] = frame
        self._emit("camera_frame", frame)

    def _on_agent_idle(self, idle: bool) -> None:
        with self.state_lock:
            self._agent_idle = bool(idle)
        self._emit("agent_idle", bool(idle))

    def _on_agent_message(self, message: BaseMessage) -> None:
        item = serialize_chat_message(message)
        with self.state_lock:
            self._chat_messages.append(item)
            del self._chat_messages[:-200]
        self._emit("agent_message", item)

    def _create_server(self) -> None:
        super()._create_server()
        assert self.sio is not None
        # The NX chat endpoint is same-origin only; the base visualizer keeps
        # its existing CORS behavior for other blueprints.
        self.sio.eio.cors_allowed_origins = None
        connect_handler = self.sio.handlers["/"]["connect"]

        async def connect(sid: str, environ: dict[str, Any], auth: Any = None) -> Any:
            with self.state_lock:
                self.vis_state.update(self._connection_state())
            return await connect_handler(sid, environ)

        self.sio.handlers["/"]["connect"] = connect
        self.sio.on("agent_send", self._on_agent_send)
        self.sio.on("memory_snapshot", self._on_memory_snapshot)
        self.sio.on("memory_search", self._on_memory_search)

    def _connection_state(self) -> dict[str, Any]:
        return {
            "agent_available": self._agent_available(),
            "agent_idle": self._agent_idle,
            "agent_messages": list(self._chat_messages),
            "memory_available": getattr(self, "_memory", None) is not None,
        }

    def _agent_available(self) -> bool:
        return self.agent.transport is not None and self.human_input.transport is not None

    async def _on_agent_send(self, _sid: str, payload: Any) -> dict[str, Any]:
        text = payload.get("text") if isinstance(payload, dict) else None
        if not isinstance(text, str) or not text.strip():
            return {"ok": False, "error": "text must be a non-empty string"}
        if len(text) > 4000:
            return {"ok": False, "error": "text must be at most 4000 characters"}
        with self.state_lock:
            if not self._agent_available():
                return {"ok": False, "error": "agent transport is not bound"}
            if self._agent_idle is not True:
                return {"ok": False, "error": "agent is not idle"}
            self._agent_idle = False
        self._emit("agent_idle", False)
        try:
            self.human_input.publish(text)
        except Exception as exc:
            with self.state_lock:
                self._agent_idle = True
            self._emit("agent_idle", True)
            return {"ok": False, "error": f"agent input publish failed: {exc}"}
        return {"ok": True}

    async def _on_memory_snapshot(self, _sid: str, _payload: Any = None) -> dict[str, Any]:
        return await self._memory_call(lambda: self._memory.get_visualization_snapshot())

    async def _on_memory_search(self, _sid: str, payload: Any) -> dict[str, Any]:
        query = payload.get("query") if isinstance(payload, dict) else None
        if not isinstance(query, str) or not query.strip():
            return {"ok": False, "error": "query must be a non-empty string"}
        if len(query) > 512:
            return {"ok": False, "error": "query must be at most 512 characters"}
        return await self._memory_call(lambda: self._memory.search_visualization(query.strip()))

    async def _memory_call(self, call: Callable[[], Any]) -> dict[str, Any]:
        memory = getattr(self, "_memory", None)
        if memory is None:
            return {"ok": False, "error": "spatial memory is not bound"}
        try:
            return {"ok": True, "data": await asyncio.to_thread(call)}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}


def _text_content(content: Any) -> str:
    if isinstance(content, str):
        return content
    if not isinstance(content, list):
        return str(content)
    parts = [
        str(part["text"])
        for part in content
        if isinstance(part, dict) and isinstance(part.get("text"), str)
    ]
    return "\n".join(parts)


def serialize_chat_message(message: BaseMessage) -> ChatMessage:
    role = {"human": "human", "ai": "ai", "tool": "tool"}.get(message.type, "tool")
    item: ChatMessage = {
        "id": str(message.id or uuid.uuid4()),
        "role": role,
        "content": _text_content(message.content),
        "tool_calls": [
            {
                "id": str(call.get("id") or ""),
                "name": str(call.get("name") or ""),
                "args": call.get("args", {}),
            }
            for call in getattr(message, "tool_calls", [])
        ],
        "ts": time.time(),
    }
    tool_call_id = getattr(message, "tool_call_id", None)
    name = getattr(message, "name", None)
    if tool_call_id:
        item["tool_call_id"] = str(tool_call_id)
    if name:
        item["name"] = str(name)
    return item


__all__ = [
    "ChatMessage",
    "MemoryRecord",
    "MemorySnapshot",
    "NXAgentMemoryWebModule",
    "SpatialVisualizationSpec",
    "serialize_chat_message",
]
