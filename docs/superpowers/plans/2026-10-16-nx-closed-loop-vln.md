# NX Closed-loop VLN Implementation Plan

**Goal:** implement the approved design in `../specs/2026-10-16-nx-closed-loop-vln-design.md`.

**Architecture:** a new navigation skill container owns bounded tasks and asynchronous
segment feedback. A separate NX blueprint composes it with existing modules.

**Tech Stack:** Python, DimOS Modules/Spec RPC, LangChain image messages, pytest mocks.

## Tasks

- [x] Freeze and push the existing direct-cmd baseline before feature work (`d5ed2b2`,
      tag `go2-nx-agentic-direct-cmd-20261016`).
- [x] Add `dimos/agents/skills/nx_closed_loop.py` and focused mock tests.
      Verify exact tag/CLIP retrieval, frame/odom checks, bounded segment goals,
      post-result image feedback, cancellation and budgets. No live robot actions.
- [x] Add `unitree_go2_nx_closed_loop.py`, explicit system prompt and opt-in launcher flag.
      Generate `all_blueprints.py` with the existing generator, never edit it manually.
- [x] Add brief usage instructions and validate composition, exposed tools and script syntax.
- [x] Commit the tested feature separately; report offline versus hardware verification clearly.

## Verification

- Six task/segment mock tests passed on an isolated Pusai snapshot, including the
  two-segment image-feedback path, stale imagery, stop/replacement, odom freshness,
  per-user-input task authorization, budgets and timeout cancellation.
- Four new blueprint/web checks and four legacy NX blueprint checks passed.
- Generated registry check passed; Ruff 0.14.3 check/format, Python compilation,
  shell syntax and `git diff --check` passed.
- Existing pytest 6.2.5 required `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` to avoid an
  incompatible globally installed AnyIO plugin; no dependency was installed.
- No paid model call, new live blueprint launch, Go2 command or gate change.
  Actual Qwen decision quality and robot navigation remain unverified.

The user already requested execution of the agreed architecture; no additional design
approval or worktree is required. Existing untracked historical probes remain untouched.

