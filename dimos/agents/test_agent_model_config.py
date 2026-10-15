# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from typing import Any
import sys
from types import ModuleType

import langchain.agents
import langchain.chat_models
import pytest

from dimos.agents.agent import Agent


def _capture_model(monkeypatch: pytest.MonkeyPatch, agent: Agent) -> list[Any]:
    captured: list[Any] = []
    monkeypatch.setattr(
        langchain.agents,
        "create_agent",
        lambda **kwargs: captured.append(kwargs["model"]) or object(),
    )
    monkeypatch.setattr(agent._thread, "start", lambda: None)
    agent.on_system_modules([])
    return captured


def test_model_kwargs_initialize_chat_model(monkeypatch: pytest.MonkeyPatch) -> None:
    kwargs = {"streaming": True, "reasoning_effort": "none", "timeout": 30, "max_retries": 0}
    expected_model = object()
    init_calls: list[tuple[str, dict[str, Any]]] = []

    def init_chat_model(model: str, **model_kwargs: Any) -> object:
        init_calls.append((model, model_kwargs))
        return expected_model

    monkeypatch.setattr(langchain.chat_models, "init_chat_model", init_chat_model)
    agent = Agent(model="openai:qwen-example", model_kwargs=kwargs)

    assert _capture_model(monkeypatch, agent) == [expected_model]
    assert init_calls == [("openai:qwen-example", kwargs)]
    agent.stop()


def test_default_model_string_is_passed_through(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        langchain.chat_models,
        "init_chat_model",
        lambda *args, **kwargs: pytest.fail("unexpected model initialization"),
    )
    agent = Agent()

    assert _capture_model(monkeypatch, agent) == ["gpt-4o"]
    agent.stop()


def test_ollama_model_is_still_ensured_and_passed_through(monkeypatch: pytest.MonkeyPatch) -> None:
    ensured: list[str] = []
    ollama_agent = ModuleType("dimos.agents.ollama_agent")
    monkeypatch.setattr(ollama_agent, "ensure_ollama_model", ensured.append, raising=False)
    monkeypatch.setitem(sys.modules, "dimos.agents.ollama_agent", ollama_agent)
    agent = Agent(model="ollama:qwen2.5")

    assert _capture_model(monkeypatch, agent) == ["ollama:qwen2.5"]
    assert ensured == ["qwen2.5"]
    agent.stop()


def test_model_fixture_takes_precedence_over_model_kwargs(monkeypatch: pytest.MonkeyPatch) -> None:
    import dimos.agents.testing

    fixture_model = object()
    monkeypatch.setattr(dimos.agents.testing, "MockModel", lambda **kwargs: fixture_model)
    monkeypatch.setattr(
        langchain.chat_models,
        "init_chat_model",
        lambda *args, **kwargs: pytest.fail("fixture must take precedence"),
    )
    agent = Agent(
        model="openai:qwen-example",
        model_kwargs={"streaming": True},
        model_fixture="fixture.json",
    )

    assert _capture_model(monkeypatch, agent) == [fixture_model]
    agent.stop()
