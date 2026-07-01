# First Motive Teleop Panel

A Foxglove Studio panel that teleoperates a registered robot. It is the primary,
fleet-scalable teleop input: a new operator opens a Foxglove URL — no per-operator
hardware to ship.

Each robot's surface is read from a per-robot config selected in the panel settings
(mirrors `fm_bringup`'s robot registry). Single-arm robots (OpenArm, SO101) expose one
arm. The G1-D exposes its full body: both 7-DOF arms (each on its own `servo_node`), the
wheeled base, and both Dex3 hands. Adding a robot is one entry in the `ROBOTS` map in
`src/panel.tsx`.

## What It Publishes

```
geometry_msgs/TwistStamped -> <servo>/delta_twist_cmds   Cartesian arm jog (joysticks)
control_msgs/JointJog      -> <servo>/delta_joint_cmds   per-joint arm jog
geometry_msgs/Twist        -> /cmd_vel                   wheeled base (vx + vyaw [+ vy])
std_msgs/String            -> /g1_hand_teleop/<side>/preset   hand preset (open/close/pinch)
std_msgs/Float64MultiArray -> /g1_hand_teleop/<side>/sliders  hand per-joint targets
trajectory_msgs/JointTrajectory -> /so101_gripper_controller/joint_trajectory  SO101 gripper open/close
```

Arm `<servo>` is `/servo_node` for the right arm and `/servo_node_left` for the G1-D left
arm. Arm + base commands are unitless ([-1, 1]); MoveIt Servo and the diff-drive
controller scale them. The panel re-publishes on a 50 ms timer while a control is active so
motion is continuous and stops on release. Hand presets fire once; hand sliders publish the
full 7-joint vector on change. The panel also supports keyboard-held single-joint nudges when
it has focus, which is useful for debug teleop on a laptop.

## Controls

The panel is a touch-first dashboard, built for two-thumb use on a tablet. Every active
control contributes to a shared command map; on each timer tick the panel merges all
contributions per topic, so two sticks dragged at once publish together on the same message.

```
arm Cartesian   translate pad  lin x (fwd ↑) · lin y (lateral ↔)
                Z thumb        lin z (up/down)
                rotate pad     ang y (pitch ↕) · ang z (yaw ↔)
                roll thumb     ang x (roll)
                → all four merge into one TwistStamped per arm
keyboard        Q/A joint 1 · W/S joint 2 · E/D joint 3 · R/F joint 4 · T/G joint 5
                Y/H joint 6 · U/J joint 7 (when the selected arm has them)
                O gripper open · P gripper close (SO101)
base            drive pad      vx (fwd ↑) · vyaw (turn ↔)
                vy thumb       lateral strafe (holonomic bases only)
```

A header carries a global **STOP** (clears every active control at once), a **speed scalar**
(0–1, multiplies every published magnitude), and each stick shows its live normalized value.
The per-joint arm bank and the hands (presets + finger sliders) are collapsed by default.
Joystick deadzone and the speed scalar persist in the panel settings.

## Build + Install

This is a TypeScript/React Foxglove extension, built outside the ROS workspace with
the Foxglove toolchain (Node 18+ required):

```
cd src/fm_teleop/fm_teleop_panel
npm install
npm run local-install   # builds and installs into the local Foxglove Studio
```

`npm run package` produces a `.foxe` for distributing to other operators.

> If a code change does not show up after `local-install` + a Foxglove restart, the build
> cache served a stale `dist`. Force a clean rebuild: `rm -rf dist node_modules/.cache &&
> npm run build`, then `npm run local-install`. Foxglove loads local extensions at startup,
> so quit fully (Cmd+Q) and reopen to pick up the new bundle.

## Use

1. Start the sim and Servo: `./scripts/teleop.sh --robot openarm` (default Foxglove input).
2. In Foxglove Studio (connected to `ws://localhost:8765`), add the **First Motive Teleop**
   panel and confirm it can publish (the connection must allow advertising).
3. In the panel settings, pick the robot you launched so the joint set and command frame
   match its Servo config. Tune the speed scalar and joystick deadzone there if needed.
4. Drag the Cartesian sticks to jog the arm; expand **Per-joint** for single-joint jogging.
5. For keyboard jogging, click inside the panel or the **Click to arm keyboard** button,
   then hold one joint key pair at a time. On the SO101, `O` opens the gripper and `P`
   closes it. Foxglove must keep the panel focused while you type.
6. On the SO101, the panel still drives MoveIt Servo for keyboard jogging. Near forward
   reach, `Approaching singularity`, `Singularity stop`, `Near collision`, `Collision stop`,
   or `Joint limit stop` come from Servo's safety layer, not from MuJoCo hitting a hard wall.
7. Use **Return to default** on an arm section to send that arm back to its startup pose.

Each robot's command frame must match `robot_link_command_frame` in its Servo config
(e.g. `openarm_right_base_link` for the OpenArm).
