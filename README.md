# fm_teleop

Teleop source layer. Metapackage grouping every teleop input behind one shared
command contract. Split-ready: this whole group extracts cleanly into its own repo
later.

## Sub-packages

```
fm_teleop_core    -> TeleopSource base node + command contract + retarget utils
fm_teleop_device  -> gamepad, SpaceMouse, and G1-D hand sources
fm_teleop_leader  -> leader-arm source (skeleton)
fm_teleop_vr      -> VR controller source (skeleton)
fm_teleop_vision  -> vision / hand-tracking source (skeleton)
fm_teleop_panel   -> browser Foxglove panel (npm, COLCON_IGNORE'd)
```

## Build type

`ament_cmake` metapackage (exec-depends on the ROS sub-packages; the panel is npm,
built outside colcon).

See `fm_teleop_core/README.md` for the contract every source collapses to.
