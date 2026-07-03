// Decide the panel's slider set and seed from incoming /robot_description and
// /joint_states, as a pure step so it is unit-testable outside React.
//
// The panel calls reconcile() on every render frame with the latest URDF and
// joint-state seen so far. reconcile owns two decisions:
//   1. A NEW URDF rebuilds the slider set and seeds it (from the latest
//      /joint_states if one has arrived, else zeros).
//   2. The FIRST real /joint_states after the sliders exist reseeds them, so the
//      panel opens at the robot's current pose rather than the zeros it showed
//      before the message landed.
// Any later /joint_states is ignored — the operator now owns the sliders.

import { JointStateMessage, seedPositions } from "./jsp-seed";
import { MovableJoint, parseMovableJoints } from "./urdf";

export type JspModel = {
  // The URDF string the current sliders were built from (undefined until first).
  parsedUrdf: string | undefined;
  joints: MovableJoint[];
  // Seeded from a real /joint_states yet? Guards against reseeding over operator input.
  seeded: boolean;
};

export type ReconcileOutput = {
  model: JspModel;
  // Present only when the caller should replace the slider set / seed values.
  joints?: MovableJoint[];
  seedValues?: Record<string, number>;
};

export const EMPTY_MODEL: JspModel = { parsedUrdf: undefined, joints: [], seeded: false };

export function reconcile(
  model: JspModel,
  urdf: string | undefined,
  latest: JointStateMessage | undefined,
): ReconcileOutput {
  if (urdf && urdf !== model.parsedUrdf) {
    const joints = parseMovableJoints(urdf);
    return {
      model: { parsedUrdf: urdf, joints, seeded: latest != undefined },
      joints,
      seedValues: seedPositions(joints, latest),
    };
  }
  if (!model.seeded && model.joints.length > 0 && latest) {
    return {
      model: { ...model, seeded: true },
      seedValues: seedPositions(model.joints, latest),
    };
  }
  return { model };
}
