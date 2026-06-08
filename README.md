# fm_teleop

Teleop source layer. A metapackage that owns every teleop input behind one shared
command contract. Split-ready: the whole group extracts cleanly into its own repo later.

Teleop is First Motive's largest data-capture surface — many input modalities feeding one
robot fleet. This package is the home for the **source** side of that: each modality
collapses to the same small set of standard-message channels, so the sinks in `fm_control`
(MoveIt Servo, the controllers, the hardware bridges) never need to know which device drove
them.

## Convergence Model

```
SOURCES                        CONTRACT (standard msgs)             SINKS (fm_control)
panel · gamepad · spacemouse   TwistStamped / JointJog -> Servo     controllers
hand · [leader·vr·vision]      Twist -> /cmd_vel                    + bridges
                               JointTrajectory (leader bypass)
                               String / Float64MultiArray -> hands
```

`fm_teleop_core` ships the `TeleopSource` base node that encodes this contract. Every source
subclasses it; that base class is what makes a new source one file of mapping logic.

## Layout

```
fm_teleop/            (container directory — not itself a package)
├── fm_teleop/        ament_cmake metapackage (sibling, so colcon sees the children)
├── fm_teleop_core/   TeleopSource base node + command contract + retarget utils
├── fm_teleop_device/ gamepad, SpaceMouse, and the G1-D hand mapper
├── fm_teleop_leader/ leader-arm follow (skeleton)
├── fm_teleop_vr/     VR controllers (skeleton)
├── fm_teleop_vision/ vision hand-tracking (skeleton)
└── fm_teleop_panel/  browser Foxglove panel (npm, COLCON_IGNORE'd)
```

The metapackage lives as a *sibling* of the children rather than their parent directory:
colcon prunes its crawl at any directory that is itself a package, so nesting the children
under the metapackage would hide them from the build.

## Source Status

| Source     | Package / node                     | Modality              | Contract channels                         | Status   |
| ---------- | ---------------------------------- | --------------------- | ----------------------------------------- | -------- |
| Panel      | `fm_teleop_panel`                  | browser (no HW)       | arm_twist, arm_joint, base_twist, hand_preset, hand_sliders | working  |
| Gamepad    | `fm_teleop_device/joy_to_servo`    | Xbox-style pad        | arm_twist                                 | working  |
| SpaceMouse | `fm_teleop_device/spacenav_to_servo` | 6-DOF USB           | arm_twist                                 | working  |
| Hand       | `fm_teleop_device/g1_hand_teleop`  | preset/slider mapper  | consumes hand_preset, hand_sliders        | working  |
| Leader     | `fm_teleop_leader`                 | physical leader arm   | arm_trajectory (leader bypass)            | skeleton |
| VR         | `fm_teleop_vr`                     | headset controllers   | arm_twist, base_twist, hand_preset, hand_sliders | skeleton |
| Vision     | `fm_teleop_vision`                 | camera hand-tracking  | arm_twist, hand_preset, hand_sliders      | skeleton |

The panel is the scalable spine: a new operator opens a URL, no hardware shipped. After it,
vision is the most scalable physical source (just a webcam).

### Future sources (no package yet)

Documented here so the intent is on record; each gets a package when a session implements it
(no empty packages until then): **keyboard** (debug nudges), **phone AR** (ARKit/ARCore pose),
**glove / exoskeleton** (per-finger + arm), **mocap suit** (full-body retarget).

## Command Contract

The contract is standard messages only — no custom interface package (`fm_teleop_msgs`).
`fm_teleop_core/contract.py` is the single source of truth:

| Channel          | Message                           | Default topic                  |
| ---------------- | --------------------------------- | ------------------------------ |
| `arm_twist`      | `geometry_msgs/TwistStamped`      | `/servo_node/delta_twist_cmds` |
| `arm_joint`      | `control_msgs/JointJog`           | `/servo_node/delta_joint_cmds` |
| `base_twist`     | `geometry_msgs/Twist`             | `/cmd_vel`                     |
| `hand_preset`    | `std_msgs/String`                 | per side (set at construction) |
| `hand_sliders`   | `std_msgs/Float64MultiArray`      | per side (set at construction) |
| `arm_trajectory` | `trajectory_msgs/JointTrajectory` | per arm (leader bypass)        |

## Add a Source

1. **Pick the package.** A physical device → `fm_teleop_device`. A new modality that earns
   its own home → a sibling package (clone a skeleton's shape: `package.xml`, `setup.py`,
   `setup.cfg`, `resource/`, `test/`).
2. **Subclass `TeleopSource`.** Declare params, build publishers with
   `self.contract_publisher(channel)`, subscribe to the device, map each reading onto a
   contract message in the callback. Use `fm_teleop_core.retarget` for deadzone / clamp /
   scale. Never hard-code a topic or message type — both come from the contract.
3. **Wire the launch.** Add the node to `fm_bringup/teleop.launch.py` (launch orchestration
   stays in bringup), or to a robot's `teleop_nodes` in `fm_bringup/registry.py` if it is
   robot-specific.
4. **Test + document.** Ship a smoke test and a package README stating the device → channel
   mapping; add a row to the source-status table above.

See `fm_teleop_core/README.md` for the contract and a worked source example.

## Build

`ament_cmake` metapackage over `ament_python` children. The panel is npm, built outside
colcon (`cd fm_teleop_panel && npm run local-install`).
