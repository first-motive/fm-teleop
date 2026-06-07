# fm_teleop_core

The contract every teleop source collapses to, plus the base node that encodes it.
This package holds no source of its own — it is what makes a new source one file of
mapping logic.

## The contract

Many input modalities, one robot fleet. Whatever the device, a source publishes one of
a small fixed set of standard-interface channels; the sinks in `fm_control` consume them.

```
SOURCES                        CONTRACT (standard msgs)             SINKS (fm_control)
panel · gamepad · spacemouse   TwistStamped / JointJog -> Servo     controllers
hand · [leader·vr·vision]      Twist -> /cmd_vel                    + bridges
                               JointTrajectory (leader bypass)
                               String / Float64MultiArray -> hands
```

`contract.py` is the single source of truth for that set — each channel's message type
and default topic:

| Channel          | Message                          | Default topic                    |
| ---------------- | -------------------------------- | -------------------------------- |
| `arm_twist`      | `geometry_msgs/TwistStamped`     | `/servo_node/delta_twist_cmds`   |
| `arm_joint`      | `control_msgs/JointJog`          | `/servo_node/delta_joint_cmds`   |
| `base_twist`     | `geometry_msgs/Twist`            | `/cmd_vel`                       |
| `hand_preset`    | `std_msgs/String`                | per side (set at construction)   |
| `hand_sliders`   | `std_msgs/Float64MultiArray`     | per side (set at construction)   |
| `arm_trajectory` | `trajectory_msgs/JointTrajectory`| per arm (leader bypass)          |

No custom interface package (`fm_teleop_msgs`) exists by design — standard messages only.

## Modules

```
contract.py   the channel set above: message type + default topic per channel
source.py     TeleopSource(Node): contract_publisher() + stamped_header()
retarget.py   pure device-to-command math (deadzone, clamp, clamp_vector, scale)
```

## Writing a source

```python
from fm_teleop_core import retarget
from fm_teleop_core.source import TeleopSource


class MySource(TeleopSource):
    def __init__(self):
        super().__init__("my_source")
        self._frame = self.declare_parameter("command_frame", "base_link").value
        self._pub = self.contract_publisher("arm_twist")
        self.create_subscription(MyDeviceMsg, "/my_device", self._on_device, 10)

    def _on_device(self, msg):
        twist = TwistStamped(header=self.stamped_header(self._frame))
        twist.twist.linear.x = retarget.deadzone(msg.x, 0.1)
        self._pub.publish(twist)
```

The subclass never names a topic or message type — both come from the contract.

## Build type

`ament_python`.
