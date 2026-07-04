// Parse a URDF for its movable joints and their slider bounds.
//
// The joint-state-publisher panel reads /robot_description (a std_msgs/String
// URDF), extracts every movable joint, and draws one slider per joint bounded by
// its limits. A regex parse (not DOMParser) keeps this pure so it runs unchanged
// in the browser panel and in node vitest — the launch file parses URDF the same
// way. Fixed and mimic joints carry no controllable DOF and are skipped.

export type MovableJoint = {
  name: string;
  type: string;
  lower: number;
  upper: number;
};

// Joint types that expose a controllable position. `planar` and `floating` are
// multi-DOF and have no single slider bound, so they are intentionally excluded.
const MOVABLE_TYPES = new Set(["revolute", "prismatic", "continuous"]);

// A continuous joint has no <limit> lower/upper (it spins freely). Bound its
// slider to [-pi, pi] so the control is usable; the joint itself wraps.
const CONTINUOUS_BOUND = Math.PI;

function attr(attrs: string, key: string): string | undefined {
  const match = new RegExp(`\\b${key}\\s*=\\s*"([^"]*)"`).exec(attrs);
  return match ? match[1] : undefined;
}

export function parseMovableJoints(urdf: string): MovableJoint[] {
  const joints: MovableJoint[] = [];
  // Each <joint ...> ... </joint> block. Movable joints always carry <parent>/
  // <child> children, so they are never self-closing; the greedy-safe [\s\S]*?
  // stops at the first </joint>.
  const jointBlock = /<joint\b([^>]*)>([\s\S]*?)<\/joint>/g;
  let block: RegExpExecArray | null;
  while ((block = jointBlock.exec(urdf)) !== null) {
    const [, attrs, body] = block;
    const name = attr(attrs!, "name");
    const type = attr(attrs!, "type");
    if (!name || !type || !MOVABLE_TYPES.has(type)) {
      continue;
    }
    // A mimic joint follows another joint and has no independent DOF — skip it.
    if (/<mimic\b/.test(body!)) {
      continue;
    }
    let lower: number;
    let upper: number;
    if (type === "continuous") {
      lower = -CONTINUOUS_BOUND;
      upper = CONTINUOUS_BOUND;
    } else {
      const limit = /<limit\b([^>]*?)\/?>/.exec(body!);
      const lo = limit ? attr(limit[1]!, "lower") : undefined;
      const hi = limit ? attr(limit[1]!, "upper") : undefined;
      // A revolute/prismatic joint without an explicit limit is degenerate; fall
      // back to a zero-width bound so the slider renders inert rather than crash.
      lower = lo !== undefined ? parseFloat(lo) : 0;
      upper = hi !== undefined ? parseFloat(hi) : 0;
    }
    joints.push({ name, type, lower, upper });
  }
  return joints;
}
