// Seed slider positions from the live robot, never from zeros.
//
// On mount the panel seeds each joint's slider from the latest /joint_states so
// it opens at the robot's CURRENT pose (the home pose jsp published), not at
// model-zero. A joint absent from /joint_states falls back to 0, clamped into
// its limits so the slider stays in range. This mirrors the launch-side single
// -publisher invariant: the panel starts where jsp already is, so publishing its
// seed to /joint_command changes nothing until the operator drags a slider.

import { MovableJoint } from "./urdf";

// The fields of sensor_msgs/JointState the seed reads. Everything else is unused.
export type JointStateMessage = {
  name?: string[];
  position?: number[];
};

function clamp(value: number, lower: number, upper: number): number {
  // A degenerate bound (upper <= lower, e.g. a limitless joint) can't clamp, so
  // pass the value through untouched.
  if (upper <= lower) {
    return value;
  }
  return Math.max(lower, Math.min(upper, value));
}

export function seedPositions(
  joints: MovableJoint[],
  latest: JointStateMessage | undefined,
): Record<string, number> {
  const live = new Map<string, number>();
  if (latest?.name && latest.position) {
    for (let i = 0; i < latest.name.length; i++) {
      const position = latest.position[i];
      if (typeof position === "number") {
        live.set(latest.name[i]!, position);
      }
    }
  }
  const seeded: Record<string, number> = {};
  for (const joint of joints) {
    const value = live.has(joint.name) ? live.get(joint.name)! : 0;
    seeded[joint.name] = clamp(value, joint.lower, joint.upper);
  }
  return seeded;
}
