# Changelog

## 0.0.2

- G1-D full-body teleop: the panel now exposes both arms (each on its own `servo_node`),
  the wheeled base (`/cmd_vel`, vx + vyaw), and both Dex3 hands (presets + per-joint
  sliders). Single-arm robots (openarm, so101) are unchanged.

## 0.0.1

- Initial release. Robot-aware teleop panel: publishes `TwistStamped` +
  `JointJog` to MoveIt Servo, with the joint set, command frame, and Cartesian
  availability chosen per robot in the panel settings (openarm, so101, g1_d).
