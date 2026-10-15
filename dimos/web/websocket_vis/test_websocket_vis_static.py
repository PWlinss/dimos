# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0

"""The command center serves the checkout's build rather than a stale cached UI."""

from pathlib import Path
from types import SimpleNamespace

from starlette.testclient import TestClient

from dimos.web.websocket_vis import websocket_vis_module as web_module


def test_local_command_center_build_and_assets(tmp_path: Path, monkeypatch) -> None:
    build = tmp_path / "dist-standalone"
    (build / "assets").mkdir(parents=True)
    (build / "index.html").write_text('<script src="/assets/main.js"></script>')
    (build / "assets" / "main.js").write_text("window.localBuild = true;")
    monkeypatch.setattr(web_module, "_COMMAND_CENTER_DIR", build)

    def no_cached_download(*args, **kwargs):
        raise AssertionError("A local build must not download a cached command center")

    monkeypatch.setattr(web_module, "get_data", no_cached_download)
    module = web_module.WebsocketVisModule.__new__(web_module.WebsocketVisModule)
    module._global_config = SimpleNamespace(viewer="none")
    module._create_server()
    with TestClient(module.app) as client:
        response = client.get("/command-center")
        assert response.status_code == 200
        assert "/assets/main.js" in response.text
        assert client.get("/assets/main.js").text == "window.localBuild = true;"
