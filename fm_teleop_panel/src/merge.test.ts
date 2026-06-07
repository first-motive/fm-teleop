import { describe, expect, it } from "vitest";
import {
  Contribution,
  mergeContributions,
  toMessage,
  twistAxis,
  twistStampedAxis,
} from "./merge";

const FRAME = "torso_link";
const ARM = "/servo_node/delta_twist_cmds";

describe("mergeContributions", () => {
  it("returns one contribution per topic", () => {
    const merged = mergeContributions([
      twistStampedAxis(ARM, FRAME, "linear", "x", 1),
      twistAxis("/cmd_vel", "linear", "x", 1),
    ]);
    expect(merged).toHaveLength(2);
    expect(merged.map((c) => c.topic).sort()).toEqual(["/cmd_vel", ARM]);
  });

  it("combines translate + Z + roll + rotate into one TwistStamped", () => {
    const merged = mergeContributions([
      twistStampedAxis(ARM, FRAME, "linear", "x", 0.5), // translate fwd
      twistStampedAxis(ARM, FRAME, "linear", "y", -0.3), // translate lateral
      twistStampedAxis(ARM, FRAME, "linear", "z", 0.2), // Z thumb
      twistStampedAxis(ARM, FRAME, "angular", "x", 0.4), // roll thumb
      twistStampedAxis(ARM, FRAME, "angular", "z", -0.1), // rotate yaw
      twistStampedAxis(ARM, FRAME, "angular", "y", 0.6), // rotate pitch
    ]);
    expect(merged).toHaveLength(1);
    const c = merged[0]!;
    if (c.kind !== "twistStamped") throw new Error("expected twistStamped");
    expect(c.linear).toEqual({ x: 0.5, y: -0.3, z: 0.2 });
    expect(c.angular).toEqual({ x: 0.4, y: 0.6, z: -0.1 });
  });

  it("sums collisions on the same axis rather than overwriting", () => {
    const merged = mergeContributions([
      twistStampedAxis(ARM, FRAME, "linear", "x", 0.4),
      twistStampedAxis(ARM, FRAME, "linear", "x", 0.3),
    ]);
    const c = merged[0]!;
    if (c.kind !== "twistStamped") throw new Error("expected twistStamped");
    expect(c.linear.x).toBeCloseTo(0.7);
  });

  it("sums two base twist contributions on /cmd_vel (drive vx + strafe vy)", () => {
    // Mirrors BaseJoystick: drive stick writes vx + vyaw, the strafe thumb writes
    // vy; both land on /cmd_vel and must merge into one Twist.
    const merged = mergeContributions([
      { kind: "twist", topic: "/cmd_vel", linear: { x: 0.6, y: 0, z: 0 }, angular: { x: 0, y: 0, z: -0.4 } },
      { kind: "twist", topic: "/cmd_vel", linear: { x: 0, y: 0.3, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
    ]);
    expect(merged).toHaveLength(1);
    const c = merged[0]!;
    if (c.kind !== "twist") throw new Error("expected twist");
    expect(c.linear).toEqual({ x: 0.6, y: 0.3, z: 0 });
    expect(c.angular.z).toBeCloseTo(-0.4);
  });

  it("merges jointJog velocities by name, summing on collision", () => {
    const a: Contribution = {
      kind: "jointJog",
      topic: "/jog",
      frame: FRAME,
      velocities: { j1: 1, j2: 0.5 },
    };
    const b: Contribution = {
      kind: "jointJog",
      topic: "/jog",
      frame: FRAME,
      velocities: { j2: 0.5, j3: -1 },
    };
    const merged = mergeContributions([a, b]);
    const c = merged[0]!;
    if (c.kind !== "jointJog") throw new Error("expected jointJog");
    expect(c.velocities).toEqual({ j1: 1, j2: 1, j3: -1 });
  });

  it("does not mutate source contributions", () => {
    const src = twistStampedAxis(ARM, FRAME, "linear", "x", 1);
    mergeContributions([src, twistStampedAxis(ARM, FRAME, "linear", "x", 1)]);
    if (src.kind !== "twistStamped") throw new Error("expected twistStamped");
    expect(src.linear.x).toBe(1);
  });
});

describe("toMessage", () => {
  const stamp = { sec: 7, nsec: 0 };

  it("builds a TwistStamped with header frame", () => {
    const msg = toMessage(twistStampedAxis(ARM, FRAME, "linear", "x", 1), stamp) as {
      header: { frame_id: string; stamp: typeof stamp };
      twist: { linear: { x: number } };
    };
    expect(msg.header.frame_id).toBe(FRAME);
    expect(msg.header.stamp).toEqual(stamp);
    expect(msg.twist.linear.x).toBe(1);
  });

  it("builds a headerless Twist for the base", () => {
    const msg = toMessage(twistAxis("/cmd_vel", "angular", "z", 0.5), stamp) as {
      angular: { z: number };
      header?: unknown;
    };
    expect(msg.angular.z).toBe(0.5);
    expect(msg.header).toBeUndefined();
  });

  it("builds JointJog with aligned names and velocities", () => {
    const msg = toMessage(
      { kind: "jointJog", topic: "/jog", frame: FRAME, velocities: { j1: 0.2, j2: -0.3 } },
      stamp,
    ) as { joint_names: string[]; velocities: number[] };
    expect(msg.joint_names).toEqual(["j1", "j2"]);
    expect(msg.velocities).toEqual([0.2, -0.3]);
  });
});
