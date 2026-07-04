import { describe, expect, it } from "vitest";

import { parseMovableJoints } from "./urdf";

const URDF = `<?xml version="1.0"?>
<robot name="demo">
  <link name="base"/>
  <link name="a"/>
  <link name="b"/>
  <joint name="fixed_joint" type="fixed">
    <parent link="base"/><child link="a"/>
  </joint>
  <joint name="rev_joint" type="revolute">
    <parent link="a"/><child link="b"/>
    <limit lower="-1.5" upper="2.0" effort="10" velocity="1"/>
  </joint>
  <joint name="prism_joint" type="prismatic">
    <parent link="a"/><child link="b"/>
    <limit effort="5" velocity="1" lower="0" upper="0.3"/>
  </joint>
  <joint name="wheel_joint" type="continuous">
    <parent link="a"/><child link="b"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>`;

describe("parseMovableJoints", () => {
  it("returns only movable joints, skipping fixed", () => {
    const joints = parseMovableJoints(URDF);
    expect(joints.map((j) => j.name)).toEqual(["rev_joint", "prism_joint", "wheel_joint"]);
  });

  it("reads revolute limits regardless of attribute order", () => {
    const joints = parseMovableJoints(URDF);
    const rev = joints.find((j) => j.name === "rev_joint")!;
    expect(rev).toMatchObject({ type: "revolute", lower: -1.5, upper: 2.0 });
    // prismatic lists lower/upper after effort/velocity — attribute order must not matter.
    const prism = joints.find((j) => j.name === "prism_joint")!;
    expect(prism).toMatchObject({ type: "prismatic", lower: 0, upper: 0.3 });
  });

  it("bounds a continuous joint to [-pi, pi]", () => {
    const wheel = parseMovableJoints(URDF).find((j) => j.name === "wheel_joint")!;
    expect(wheel.lower).toBeCloseTo(-Math.PI);
    expect(wheel.upper).toBeCloseTo(Math.PI);
  });

  it("skips a mimic joint (no independent DOF)", () => {
    const urdf = `<robot name="m">
      <joint name="driver" type="revolute">
        <limit lower="0" upper="1"/>
      </joint>
      <joint name="follower" type="revolute">
        <mimic joint="driver" multiplier="1"/>
        <limit lower="0" upper="1"/>
      </joint>
    </robot>`;
    expect(parseMovableJoints(urdf).map((j) => j.name)).toEqual(["driver"]);
  });

  it("returns [] for a URDF with no movable joints", () => {
    expect(parseMovableJoints('<robot name="x"><link name="l"/></robot>')).toEqual([]);
  });
});
