import { describe, expect, it } from "vitest";

import { seedPositions } from "./jsp-seed";
import { MovableJoint } from "./urdf";

const JOINTS: MovableJoint[] = [
  { name: "a", type: "revolute", lower: -1, upper: 1 },
  { name: "b", type: "revolute", lower: 0, upper: 2 },
  { name: "c", type: "continuous", lower: -Math.PI, upper: Math.PI },
];

describe("seedPositions", () => {
  it("seeds each joint from the latest /joint_states", () => {
    const seeded = seedPositions(JOINTS, {
      name: ["a", "b", "c"],
      position: [0.5, 1.5, 3.0],
    });
    expect(seeded).toEqual({ a: 0.5, b: 1.5, c: 3.0 });
  });

  it("falls back to 0 for a joint absent from /joint_states, clamped into range", () => {
    // `b` is missing from the message and its range excludes 0 → clamps to lower.
    const seeded = seedPositions(JOINTS, { name: ["a"], position: [0.2] });
    expect(seeded.a).toBe(0.2);
    expect(seeded.b).toBe(0); // 0 is inside [0, 2]
    expect(seeded.c).toBe(0);
  });

  it("clamps 0 up to the lower bound when the range excludes 0", () => {
    const joints: MovableJoint[] = [{ name: "j", type: "revolute", lower: 0.5, upper: 1.5 }];
    expect(seedPositions(joints, undefined).j).toBe(0.5);
  });

  it("clamps a live value that exceeds the joint limits", () => {
    const seeded = seedPositions(JOINTS, { name: ["a"], position: [9.9] });
    expect(seeded.a).toBe(1); // clamped to upper
  });

  it("all-zero with no message and 0 inside every range (never crashes)", () => {
    expect(seedPositions(JOINTS, undefined)).toEqual({ a: 0, b: 0, c: 0 });
  });

  it("ignores a message with mismatched name/position lengths gracefully", () => {
    // position shorter than name: the missing index is undefined → joint falls back.
    const seeded = seedPositions(JOINTS, { name: ["a", "b"], position: [0.3] });
    expect(seeded.a).toBe(0.3);
    expect(seeded.b).toBe(0);
  });
});
