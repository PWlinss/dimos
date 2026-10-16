# Go2 NX agentic on Pusai

This starts `unitree-go2-nx-agentic`, the camera decoder, and the Command Center at `http://localhost:7779`. Rerun is disabled. Navigation and keyboard velocity commands publish directly to `/dimos/cmd_vel`, using the existing Go2 rate limits, timeout protection, and movement gates. No extra topic-forwarding process is needed. This launcher does not change the Go2 movement gates.

The imported `unitree-go2-agentic-mcp-streamvln` blueprint is an experimental legacy path, not this NX deployment. Its StreamVLN movement is disabled by default; do not enable or run that blueprint on the current Go2 without a separate safety-chain review.

## One-time setup on Pusai

Use a clean DimOS checkout on Pusai with ROS Humble and the NX ROS camera dependencies installed. The launcher defaults to checkout-local `.venv`, but Pusai's ROS Humble `rclpy` needs Python 3.10; the repository `.venv` may be Python 3.12 and is not suitable. Copy `scripts/pusai_nx_agentic.env.example` to `~/.config/dimos/nx-agentic.env` and keep its `DIMOS_VENV` set to the prepared Python 3.10 environment. The sample uses Pusai's `/home/wkr/Workspaces/workspace_peiwei/projects/VLN/dimos/.venv_ros310`; set `DIMOS_EXTRA_PYTHONPATH` to prepared Python 3.10-compatible packages if that venv lacks agentic dependencies such as Chroma or ONNX Runtime. Do not use a bare `uv sync` that creates a Python 3.12 environment for this deployment.

Set `DIMOS_CLIP_MODEL_PATH` to the ONNX file, not its containing directory. Keep the CLIP processor files (`config.json`, `preprocessor_config.json`, and tokenizer assets) beside it; startup loads these assets locally without contacting Hugging Face. The sample points to Pusai's `data/models_clip/model.onnx`. The example contains only non-secret host configuration.

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

## Opt-in closed-loop indoor VLN

The old `--agentic` blueprint is unchanged. The separate `unitree-go2-nx-closed-loop`
blueprint adds candidate retrieval, short navigation segments, and result/image feedback.
It uses the same Command Center, persistent spatial memory, `odom` frame, A* planner
and direct `/dimos/cmd_vel` transport. It does not add a relay or change robot gates.

When ready for a supervised deployment, stop the current instance before switching:

```bash
scripts/start_pusai_nx_bridge.sh stop --agentic
scripts/start_pusai_nx_bridge.sh start --closed-loop
scripts/start_pusai_nx_bridge.sh status --closed-loop
scripts/start_pusai_nx_bridge.sh verify --closed-loop
```

Send a normal request such as "go home" or "go to the storage box". The Agent first
retrieves candidates without motion, then selects a candidate ID for one segment.
After each segment, the module sends the outcome, actual pose and a fresh camera image
back to the Agent. The Agent decides whether to continue or explicitly finish.
Ambiguous matches require user clarification. Saved poses are historical observation
positions, not estimated object positions; this does not implement precise visual servoing.

Defaults bound each task to six segments, at most 2 m straight-line goal displacement
per segment, 45 seconds per segment and 180 seconds overall. A* paths may be longer
than the straight-line displacement. Failure, stale odometry, missing fresh imagery,
cancellation or exhausted budget ends continuation. "stop" cancels the current task.
Only a new user message permits starting a new task; result callbacks cannot reset
the step/time budget by silently calling `navigate_with_text` again.
The existing green Stop button cancels the task directly, without waiting for Qwen.
The new code must pass offline mock tests before hardware deployment; mock success
does not establish real-robot visual goal recognition or navigation accuracy.
