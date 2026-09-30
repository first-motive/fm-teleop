# fm_teleop_so101

The always-on stack for a LeRobot SO-101 leader/follower pair: one process that
owns both arms and runs teleoperation and recording, so `fm-robot-agent` can
drive the pair the way it drives the Axol through Almond's server.

## Why

LeRobot's `lerobot-teleoperate` and `lerobot-record` are one-shot CLIs that take
episode boundaries from the keyboard. A fleet robot needs a process that is
always up, holds the serial buses, and takes its orders from the agent. This is
that process, and nothing more: the control loop is LeRobot's own `record_loop`,
and the episodes are LeRobot's own v3 dataset writer. What this package adds is
the mode switch and a loopback HTTP API.

It is a uv project, not a ROS package (`COLCON_IGNORE`): no ROS graph runs on an
SO-101 robot host.

## What

```
  fm-robot-agent (so101 adapter)
        │ HTTP, 127.0.0.1 only
        ▼
  server ── Stack ── worker thread
                      idle:     torque off, joints polled for telemetry
                      teleop:   LeRobot record_loop, leader → follower
                      recording: the same loop, frames into <root>/<dataset>
        │                        │
   SO-101 leader (bus)     SO-101 follower (bus, cameras)
```

| Route | Body | Effect |
| --- | --- | --- |
| `GET /status` | — | mode, joints, open take, fps, data root |
| `POST /mode` | `{"mode": "idle"\|"teleop"}` | switch; `idle` turns follower torque off |
| `POST /record` | `{"action": "start", "dataset": d, "task": t}` or `{"action": "stop"}` | start or end one episode |
| `POST /stop` | — | end any take (saved), torque off, mode idle |

## Run

```bash
uv run fm-teleop-so101 \
  --leader-port /dev/serial/by-id/<leader> --follower-port /dev/serial/by-id/<follower> \
  --leader-id fm_rob_03_leader --follower-id fm_rob_03_follower \
  --owner fm-rob-03 --root <data-root>/lerobot
```

Both arms must have been calibrated once with `lerobot-calibrate`, on any host;
the calibration lives in the motors and a new host adopts it. Ports and ids are per host: pass them from
the host's unit environment, never from source.

## Failure Modes

Written before the code; each line names the behavior that handles it.

| Failure | Handling |
| --- | --- |
| This host has no calibration file for an arm | Adopt the calibration from the arm's own motor EEPROM, where `lerobot-calibrate` wrote it, and save the file. The arms move between hosts without recalibration. |
| An arm's motors still hold factory values (offset 0, full range, not the wrist roll) | Refuse to start and name `lerobot-calibrate` with the id. Never run LeRobot's interactive calibration from a service. |
| The motors disagree with this host's calibration file | Refuse to start: the file is stale (the arm was recalibrated elsewhere). Delete the file to adopt from the motors. |
| A serial port is missing or held by another process | Startup fails with LeRobot's error; the supervisor restarts it. |
| A bus error or unplugged arm during the loop | Save the open take's frames, try to turn follower torque off, log, exit non-zero. Startup then fails loudly while the arm is gone. |
| Torque off (`idle`, `stop`, exit) with the follower raised | The arm drops under gravity. Put it at rest before `idle`; `stop` accepts the drop because ending motion comes first. |
| Teleop starts with leader and follower far apart | `--max-relative-target` caps each step, so the follower ramps instead of jumping. |
| `record start` while idle | Refused: switch to teleop first, so recording never starts motion by itself. |
| `record start` during a take, or `stop` with no take | Refused with the current state. |
| `record start` with no task text | Refused: a take without a task sentence trains a policy that ignores the instruction. |
| A dataset name with `/`, `..`, or other path characters | Refused before any path is built. |
| An episode stopped before any frame was captured | The empty buffer is cleared, not saved. |
| A take interrupted by `stop` | Saved: a failed take is evidence, and the recorder never deletes one. |
| The dataset left open after a take | Every take ends with `finalize()`, so the files on disk are always valid for readers. |
| The API reached from another host | It binds `127.0.0.1` only; the agent on the same host is its one client. |
| An oversized or malformed request body | Rejected with 400 before it is parsed. |
| Two requests at once | Requests only set the wanted state under a lock; the worker thread alone touches the hardware. |

`stop` is torque-off, not an emergency stop. The SO-101 has no e-stop; the
physical stop is the follower's power supply.
