# web-controller

Drives robot setpoints from a browser so you can exercise a dataflow by hand.
On each `tick` the node publishes the latest TCP and gripper targets onto the
dora bus. The targets come from a browser page whose buttons mutate a shared
state, and the node also shows each arm's live actual pose fed back over
`<arm>_tcp_pose`. `send_output` is only ever called from the dora loop, so the
web thread just reads/writes that shared state under a lock.

See [../../docs/node_development.md](../../docs/node_development.md) for the node
pattern (tick-driven publish, `status` liveness, drop-oldest setpoints).

It serves a **closed-loop** control page built with [NiceGUI](https://nicegui.io)
(pure Python). The node subscribes to each arm's actual TCP pose
(`<arm>_tcp_pose`) and shows the **real live values** (~5 Hz); **−/+ buttons**
nudge a commanded target (seeded from the current pose, so the first move isn't a
jump), republished at 30 Hz as `<arm>_tcp_target`. A **gripper** row nudges
`<arm>_gripper_target` (metres, clamped). Open the printed `http://HOST:PORT` URL
(run the dataflow **without** `--stop-after` so it stays up).

The page also serves **recording + shutdown** controls: a **task** field and
**Start/Finish episode** buttons (published on `episode_control` as
`start`/`finish`/`task=<text>`), and a **Disconnect** button (published once on
`command` as `disconnect`, after which targets stop so the robot can fold to sleep
and the dataflow then stops).

The NiceGUI page is mounted on a small FastAPI app that also exposes a JSON
control surface for scripting/tests: `GET /state` · `POST /nudge {arm, field, dir}`
· `POST /gripper {arm, dir}` · `POST /task {text}` · `POST /episode {cmd}` ·
`POST /disconnect`. The browser talks to this server over ordinary HTTP —
deliberately **off** the dora bus.

## I/O

- **Inputs:** `tick` (publishes the latest `command` bundle each tick, teleop stage
  only); `program_state` (stops on `disconnect`; any stage change resets the target
  anchors); `<part>_tcp_pose` + `<part>_solution_pose` feedback (one per arm part).
  Wire pose feedback `queue_size: 1` + `drop_oldest`.
- **Outputs:** `command` (`float64` bundle per the descriptor layout, manual mode),
  `episode_control` (utf8 `start`/`finish`/`task=<text>`),
  `robot_command` (utf8 `disconnect`), `node_state` (edge-triggered).

## Config (env)

`HOST` (default `127.0.0.1`), `PORT` (default `8000`), `ARMS` (comma-separated arm
ids, default `left,right`; one arm → just `left`), `POS_STEP` (m/click, `0.01`),
`ROT_STEP` (rad/click, `0.05`), `GRIPPER_STEP` (m/click, `0.004`),
`GRIPPER_OPEN`/`GRIPPER_CLOSED` (range, `0.044`/`0`).

## Tests

No unit tests here on purpose (tests are only for genuinely non-trivial pure
logic — see docs/node_development.md); the e2e smoke
(`cd dataflows && uv run --project ../nodes/lerobot pytest tests -q`) exercises
this node in the full graph.
