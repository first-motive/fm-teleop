// Per-widget command contributions and their merge into one message per topic.
//
// Each active widget (a held button, a dragged joystick) contributes one
// Contribution keyed by a stable widget id. The repeat timer merges all active
// contributions — many widgets can target the same topic at once (two-thumb
// teleop) — into a single message per topic, then publishes.
//
// Merge rule per topic: twist vectors sum component-wise; jointJog velocities
// merge by joint name, summing on collision. Contributions on one topic are
// always the same kind by construction (servoTopic is always twistStamped, the
// base cmd_vel is always twist, jointTopic is always jointJog).

export type Stamp = { sec: number; nsec: number };
export type Vec3 = { x: number; y: number; z: number };

export type Contribution =
  | { kind: "twistStamped"; topic: string; frame: string; linear: Vec3; angular: Vec3 }
  | { kind: "twist"; topic: string; linear: Vec3; angular: Vec3 }
  | { kind: "jointJog"; topic: string; frame: string; velocities: Record<string, number> };

function zero(): Vec3 {
  return { x: 0, y: 0, z: 0 };
}

function addVec(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

// Combine all active contributions into one merged contribution per topic.
// Iteration order is the caller's; merge is commutative, so order is irrelevant.
export function mergeContributions(contribs: Iterable<Contribution>): Contribution[] {
  const byTopic = new Map<string, Contribution>();
  for (const c of contribs) {
    const existing = byTopic.get(c.topic);
    if (!existing) {
      byTopic.set(c.topic, cloneContribution(c));
      continue;
    }
    mergeInto(existing, c);
  }
  return Array.from(byTopic.values());
}

function cloneContribution(c: Contribution): Contribution {
  switch (c.kind) {
    case "twistStamped":
      return { ...c, linear: { ...c.linear }, angular: { ...c.angular } };
    case "twist":
      return { ...c, linear: { ...c.linear }, angular: { ...c.angular } };
    case "jointJog":
      return { ...c, velocities: { ...c.velocities } };
  }
}

// Fold `add` into `acc` in place. Both are the same kind (same topic).
function mergeInto(acc: Contribution, add: Contribution): void {
  if (acc.kind === "twistStamped" && add.kind === "twistStamped") {
    acc.linear = addVec(acc.linear, add.linear);
    acc.angular = addVec(acc.angular, add.angular);
  } else if (acc.kind === "twist" && add.kind === "twist") {
    acc.linear = addVec(acc.linear, add.linear);
    acc.angular = addVec(acc.angular, add.angular);
  } else if (acc.kind === "jointJog" && add.kind === "jointJog") {
    for (const [joint, v] of Object.entries(add.velocities)) {
      acc.velocities[joint] = (acc.velocities[joint] ?? 0) + v;
    }
  }
}

// Scale a contribution's magnitude by `factor` (the panel speed scalar, 0..1).
// Returns a new contribution; the source is untouched.
export function scaleContribution(c: Contribution, factor: number): Contribution {
  switch (c.kind) {
    case "twistStamped":
    case "twist": {
      const linear = { x: c.linear.x * factor, y: c.linear.y * factor, z: c.linear.z * factor };
      const angular = { x: c.angular.x * factor, y: c.angular.y * factor, z: c.angular.z * factor };
      return { ...c, linear, angular };
    }
    case "jointJog": {
      const velocities: Record<string, number> = {};
      for (const [joint, v] of Object.entries(c.velocities)) {
        velocities[joint] = v * factor;
      }
      return { ...c, velocities };
    }
  }
}

// Build the ROS message for a merged contribution, stamped now.
export function toMessage(c: Contribution, stamp: Stamp): unknown {
  switch (c.kind) {
    case "twistStamped":
      return {
        header: { stamp, frame_id: c.frame },
        twist: { linear: c.linear, angular: c.angular },
      };
    case "twist":
      return { linear: c.linear, angular: c.angular };
    case "jointJog": {
      const joint_names = Object.keys(c.velocities);
      return {
        header: { stamp, frame_id: c.frame },
        joint_names,
        velocities: joint_names.map((j) => c.velocities[j]),
        displacements: [],
        duration: 0,
      };
    }
  }
}

// Single-component twist contributions, used when one widget drives one axis.
export function twistStampedAxis(
  topic: string,
  frame: string,
  axis: "linear" | "angular",
  field: keyof Vec3,
  value: number,
): Contribution {
  const linear = zero();
  const angular = zero();
  (axis === "linear" ? linear : angular)[field] = value;
  return { kind: "twistStamped", topic, frame, linear, angular };
}

export function twistAxis(
  topic: string,
  axis: "linear" | "angular",
  field: keyof Vec3,
  value: number,
): Contribution {
  const linear = zero();
  const angular = zero();
  (axis === "linear" ? linear : angular)[field] = value;
  return { kind: "twist", topic, linear, angular };
}
