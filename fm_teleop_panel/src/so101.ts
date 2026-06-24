import { Stamp } from "./merge";

export const SO101_GRIPPER_TOPIC = "/so101_gripper_controller/joint_trajectory";
export const SO101_GRIPPER_JOINT = "gripper";
export const SO101_GRIPPER_OPEN = 1.5;
export const SO101_GRIPPER_CLOSED = -0.17;
export const SO101_GRIPPER_KEYS = {
  open: "O",
  close: "P",
} as const;

export type So101GripperTarget = "open" | "closed";

export function so101GripperTargetForKey(key: string): So101GripperTarget | undefined {
  const normalized = key.toLowerCase();
  if (normalized === SO101_GRIPPER_KEYS.open.toLowerCase()) {
    return "open";
  }
  if (normalized === SO101_GRIPPER_KEYS.close.toLowerCase()) {
    return "closed";
  }
  return undefined;
}

export function so101GripperKeyboardHelp(): Array<{ key: string; action: string }> {
  return [
    { key: SO101_GRIPPER_KEYS.open, action: "gripper open" },
    { key: SO101_GRIPPER_KEYS.close, action: "gripper close" },
  ];
}

export function buildSinglePointTrajectory(
  frame: string,
  jointNames: string[],
  positions: number[],
  stamp: Stamp,
  durationSec: number,
): {
  header: { stamp: Stamp; frame_id: string };
  joint_names: string[];
  points: Array<{
    positions: number[];
    velocities: [];
    accelerations: [];
    effort: [];
    time_from_start: { sec: number; nanosec: number };
  }>;
} {
  const sec = Math.max(0, Math.floor(durationSec));
  const nanosec = Math.max(0, Math.round((durationSec - sec) * 1e9));
  return {
    header: { stamp, frame_id: frame },
    joint_names: jointNames,
    points: [
      {
        positions,
        velocities: [],
        accelerations: [],
        effort: [],
        time_from_start: { sec, nanosec },
      },
    ],
  };
}

export function buildSo101GripperTrajectory(
  target: So101GripperTarget,
  stamp: Stamp,
  frame = "base_link",
): ReturnType<typeof buildSinglePointTrajectory> {
  return buildSinglePointTrajectory(
    frame,
    [SO101_GRIPPER_JOINT],
    [target === "open" ? SO101_GRIPPER_OPEN : SO101_GRIPPER_CLOSED],
    stamp,
    1,
  );
}
