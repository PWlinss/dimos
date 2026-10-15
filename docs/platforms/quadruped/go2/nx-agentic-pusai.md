# Go2 NX agentic on Pusai

This starts `unitree-go2-nx-agentic`, the camera decoder, and the Command Center at `http://localhost:7779`. Rerun is disabled. Startup sends no motion command; `DIMOS_NX_CMD_VEL_TOPIC` defaults to the isolated `/dimos/agentic_probe/cmd_vel` topic. Do not remap it to a physical controller topic.

## One-time setup on Pusai

Use a clean DimOS checkout on Pusai with ROS Humble and the NX ROS camera dependencies installed. The launcher defaults to checkout-local `.venv`, but Pusai's ROS Humble `rclpy` needs Python 3.10; the repository `.venv` may be Python 3.12 and is not suitable. Copy `scripts/pusai_nx_agentic.env.example` to `~/.config/dimos/nx-agentic.env` and keep its `DIMOS_VENV` set to the prepared Python 3.10 environment. The sample uses Pusai's `/home/wkr/Workspaces/workspace_peiwei/projects/VLN/dimos/.venv_ros310`; set `DIMOS_EXTRA_PYTHONPATH` to prepared Python 3.10-compatible packages if that venv lacks agentic dependencies such as Chroma or ONNX Runtime. Do not use a bare `uv sync` that creates a Python 3.12 environment for this deployment.

Set `DIMOS_CLIP_MODEL_PATH` to the ONNX file, not its containing directory. The sample points to Pusai's existing `data/models_clip/model.onnx`. The example contains only non-secret host configuration.

Create the Qwen credentials file separately at `~/.config/dimos/qwen.env`, readable only by your account, with `QWEN_API_KEY=...` and optionally `QWEN_MODEL=...` and `QWEN_BASE_URL=...`. The launcher reads credentials only from this fixed path; do not put secrets in the example or the non-secret settings file.

Build the ignored standalone frontend once, explicitly, from the checkout:

```bash
scripts/prepare_pusai_nx_agentic.sh
```

This runs `npm ci` and `npm run build:standalone`; it does not run automatically during startup. It requires network access to the configured npm registry.

## Start and inspect

```bash
scripts/start_pusai_nx_bridge.sh start --agentic
scripts/start_pusai_nx_bridge.sh status --agentic
scripts/start_pusai_nx_bridge.sh verify --agentic
scripts/start_pusai_nx_bridge.sh logs --agentic
scripts/start_pusai_nx_bridge.sh stop --agentic
```

Agentic verification requires camera frames by default, as well as NX pointcloud and odometry. Pass `DIMOS_NX_REQUIRE_CAMERA=false` only when intentionally verifying without camera frames. The launcher checks the Qwen file, CLIP model file, and frontend build before starting. It may request preconfigured passwordless sudo once to set local multicast and socket-buffer settings. Existing non-agentic `--raw` and `--map` commands remain supported.
