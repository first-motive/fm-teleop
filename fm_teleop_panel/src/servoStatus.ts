export const SERVO_STATUS_LABELS: Record<number, string> = {
  [-1]: "Invalid",
  0: "Ready",
  1: "Approaching singularity",
  2: "Singularity stop",
  3: "Near collision",
  4: "Collision stop",
  5: "Joint limit stop",
  6: "Leaving singularity",
};

export const SERVO_STATUS_TONES: Record<number, "neutral" | "warn" | "stop"> = {
  [-1]: "warn",
  0: "neutral",
  1: "warn",
  2: "stop",
  3: "warn",
  4: "stop",
  5: "stop",
  6: "warn",
};
