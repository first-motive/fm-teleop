import { describe, expect, it } from "vitest";

import {
  buildSinglePointTrajectory,
  buildSo101GripperTrajectory,
  so101GripperKeyboardHelp,
  so101GripperTargetForKey,
} from "./so101";

describe("so101 gripper helpers", () => {
  it("maps keyboard keys to gripper targets", () => {
    expect(so101GripperTargetForKey("o")).toBe("open");
    expect(so101GripperTargetForKey("P")).toBe("closed");
    expect(so101GripperTargetForKey("x")).toBeUndefined();
  });

  it("builds keyboard help entries", () => {
    expect(so101GripperKeyboardHelp()).toEqual([
      { key: "O", action: "gripper open" },
      { key: "P", action: "gripper close" },
    ]);
  });

  it("builds a single-point trajectory for the gripper open command", () => {
    const msg = buildSo101GripperTrajectory("open", { sec: 7, nsec: 9 });
    expect(msg.header).toEqual({ stamp: { sec: 7, nsec: 9 }, frame_id: "base_link" });
    expect(msg.joint_names).toEqual(["gripper"]);
    expect(msg.points).toEqual([
      {
        positions: [1.5],
        velocities: [],
        accelerations: [],
        effort: [],
        time_from_start: { sec: 1, nanosec: 0 },
      },
    ]);
  });

  it("builds a closed gripper target from the SRDF default", () => {
    const msg = buildSo101GripperTrajectory("closed", { sec: 1, nsec: 2 });
    expect(msg.points[0]?.positions).toEqual([-0.17]);
  });

  it("builds a generic one-point trajectory with fractional timing", () => {
    const msg = buildSinglePointTrajectory("frame_a", ["j1"], [0.4], { sec: 3, nsec: 4 }, 1.25);
    expect(msg).toEqual({
      header: { stamp: { sec: 3, nsec: 4 }, frame_id: "frame_a" },
      joint_names: ["j1"],
      points: [
        {
          positions: [0.4],
          velocities: [],
          accelerations: [],
          effort: [],
          time_from_start: { sec: 1, nanosec: 250000000 },
        },
      ],
    });
  });
});
