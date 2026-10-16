# NX closed-loop indoor VLN

Approved scope: reuse the existing NX stack and official observe/navigation-result patterns,
without migrating upstream main or changing the running robot deployment.

## Composition

- Preserve `unitree-go2-nx-agentic` and its locked Git tag.
- Add `unitree-go2-nx-closed-loop`, with the same camera, TF, memory, costmap,
  A* and direct `/dimos/cmd_vel` transport. No extra command relay or gate.
- Replace the legacy navigation skills with a bounded closed-loop container.
- Keep Qwen cloud configuration and Command Center unchanged.

## Task contract

1. `navigate_with_text(query)` begins an explicit task and returns candidates, without moving.
2. `search_memory(query)` returns bounded candidates with opaque IDs, source, score,
   observation pose and coordinate frame. Match exact saved names before CLIP search.
3. `navigate_to_candidate(candidate_id)` accepts only a candidate from the active task,
   validates fresh odom/frame/finite values/confidence, and sends one short odom goal.
   Clamp displacement to a configurable segment length (default 2 m).
4. A background waiter reports arrived/failed/timed-out/cancelled through existing
   `Agent.add_message`, attaching a new post-result camera frame when available.
   Include task ID, step, actual final pose and whether the full candidate was reached.
   No fresh frame means stop the task, not visual success.
5. Agent observes the result, continues with an existing/retrieved candidate, or explicitly
   finishes the task. Arrival at a coordinate must never be called visual goal confirmation.

## Limits and cancellation

Enforce a configurable step budget (default 6), per-segment timeout (45 s), overall
task deadline (180 s), one active segment, and cancellation in code. Stop on stale
odom, missing frames, terminal errors or exhausted budget. Old task callbacks must
not resume cancelled/replaced tasks. Do not block the Agent thread waiting for motion.
Keep manual tag/stop/state tools; provide `observe` and an explicit finish tool.
No automatic exploration, arbitrary raw velocity, dynamic Sport actions, or guessed
object 3D coordinates. Candidate poses are historical robot observation poses.

## Verification and deployment

Use fake memory, odom, camera, planner and Agent inbox to exercise retrieval, multiple
segments, callback images, terminal failures, stale inputs, bounds and cancellation.
No robot ROS publication or paid model call in tests. Test blueprint composition and
tool exposure. Add an opt-in launcher flag; do not restart the live deployment.

