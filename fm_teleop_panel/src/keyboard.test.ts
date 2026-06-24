import { describe, expect, it } from "vitest";

import { keyboardBindingForKey, keyboardHelpForArm } from "./keyboard";

const ARM = {
  key: "arm",
  jointTopic: "/servo_node/delta_joint_cmds",
  commandFrame: "base_link",
  joints: ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
};

describe("keyboardBindingForKey", () => {
  it("maps lowercase keys to single-joint jog contributions", () => {
    const binding = keyboardBindingForKey(ARM, "q");
    expect(binding?.id).toBe("kbd-arm-shoulder_pan-pos");
    if (!binding || binding.contribution.kind !== "jointJog") {
      throw new Error("expected jointJog binding");
    }
    expect(binding.contribution.velocities).toEqual({ shoulder_pan: 1 });
  });

  it("maps uppercase keys too", () => {
    const binding = keyboardBindingForKey(ARM, "F");
    expect(binding?.label).toBe("wrist_flex -");
    if (!binding || binding.contribution.kind !== "jointJog") {
      throw new Error("expected jointJog binding");
    }
    expect(binding.contribution.velocities).toEqual({ wrist_flex: -1 });
  });

  it("ignores unknown keys", () => {
    expect(keyboardBindingForKey(ARM, "x")).toBeUndefined();
  });

  it("builds help entries from the arm joint list", () => {
    expect(keyboardHelpForArm(ARM)).toEqual([
      { key: "Q / A", action: "shoulder_pan" },
      { key: "W / S", action: "shoulder_lift" },
      { key: "E / D", action: "elbow_flex" },
      { key: "R / F", action: "wrist_flex" },
      { key: "T / G", action: "wrist_roll" },
    ]);
  });
});
