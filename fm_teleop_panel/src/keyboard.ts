import { Contribution } from "./merge";

export type KeyboardArm = {
  key: string;
  jointTopic: string;
  commandFrame: string;
  joints: string[];
};

export type KeyboardBinding = {
  id: string;
  label: string;
  contribution: Contribution;
};

const JOINT_KEY_PAIRS = [
  ["q", "a"],
  ["w", "s"],
  ["e", "d"],
  ["r", "f"],
  ["t", "g"],
  ["y", "h"],
  ["u", "j"],
] as const;

export function keyboardBindingForKey(arm: KeyboardArm, key: string): KeyboardBinding | undefined {
  const normalized = key.toLowerCase();
  for (let i = 0; i < arm.joints.length && i < JOINT_KEY_PAIRS.length; i++) {
    const joint = arm.joints[i]!;
    const [positiveKey, negativeKey] = JOINT_KEY_PAIRS[i]!;
    if (normalized !== positiveKey && normalized !== negativeKey) {
      continue;
    }
    const velocity = normalized === positiveKey ? 1 : -1;
    const suffix = normalized === positiveKey ? "pos" : "neg";
    return {
      id: `kbd-${arm.key}-${joint}-${suffix}`,
      label: `${joint} ${velocity > 0 ? "+" : "-"}`,
      contribution: {
        kind: "jointJog",
        topic: arm.jointTopic,
        frame: arm.commandFrame,
        velocities: { [joint]: velocity },
      },
    };
  }
  return undefined;
}

export function keyboardHelpForArm(arm: KeyboardArm): Array<{ key: string; action: string }> {
  return arm.joints.slice(0, JOINT_KEY_PAIRS.length).map((joint, index) => {
    const [positiveKey, negativeKey] = JOINT_KEY_PAIRS[index]!;
    return {
      key: `${positiveKey.toUpperCase()} / ${negativeKey.toUpperCase()}`,
      action: joint,
    };
  });
}
