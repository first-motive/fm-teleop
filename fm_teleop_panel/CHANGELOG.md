# Changelog

## 0.0.3

- Touch-first teleop dashboard. Cartesian arm jog and the base now drive from hand-rolled
  joysticks instead of button pairs: per arm a translate pad, Z thumb, rotate pad, and roll
  thumb merge into one `TwistStamped`; the base drives from one pad (+ a strafe thumb on
  holonomic bases). Multiple controls publish together on the same tick (two-thumb use). Adds
  a global STOP, a speed scalar, per-stick live readouts, and an adjustable joystick deadzone.
  The per-joint bank and hands collapse by default. ROS topic and schema contract unchanged.

## 0.0.2

- G1-D full-body teleop: the panel now exposes both arms (each on its own `servo_node`),
  the wheeled base (`/cmd_vel`, vx + vyaw), and both Dex3 hands (presets + per-joint
  sliders). Single-arm robots (openarm, so101) are unchanged.

## 0.0.1

- Initial release. Robot-aware teleop panel: publishes `TwistStamped` +
  `JointJog` to MoveIt Servo, with the joint set, command frame, and Cartesian
  availability chosen per robot in the panel settings (openarm, so101, g1_d).
