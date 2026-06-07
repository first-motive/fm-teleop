// First Motive teleop panel — robot-aware, full-body.
//
// Publishes the command streams MoveIt Servo + the G1-D teleop nodes consume:
//   geometry_msgs/TwistStamped -> <servo>/delta_twist_cmds   (Cartesian arm jog)
//   control_msgs/JointJog      -> <servo>/delta_joint_cmds   (per-joint arm jog)
//   geometry_msgs/Twist        -> /cmd_vel                   (wheeled base)
//   std_msgs/String            -> ~/<side>/preset            (hand preset)
//   std_msgs/Float64MultiArray -> ~/<side>/sliders           (hand per-joint)
//
// Arm + base jog commands are unitless ([-1, 1]); Servo / diff_drive scale them. Buttons
// send a sustained command while held via a repeat timer, matching the command timeout.
// Hand presets fire once; hand sliders publish the full 7-joint vector on change.
//
// The surface for each robot is read from a per-robot config (ROBOTS below) selected in
// the panel settings, mirroring fm_bringup's robot registry. Single-arm robots (OpenArm,
// SO101) expose one arm and nothing else; the G1-D exposes both arms (each on its own
// servo_node), the base, and both Dex3 hands. Adding a robot is one ROBOTS entry.

import { ExtensionContext, PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

const REPEAT_MS = 50;

// One Servo-driven arm. `commandFrame` must match servo.yaml's robot_link_command_frame;
// `joints` must match the Servo group's joints in order. `servoTopic`/`jointTopic` are the
// arm's own servo_node delta topics (the G1-D runs one servo_node per arm).
type ArmGroup = {
  key: string;
  label: string;
  servoTopic: string;
  jointTopic: string;
  commandFrame: string;
  joints: string[];
  enableCartesian: boolean;
  cartesianNote?: string;
};

// Wheeled base. vx + vyaw always; vy only when the base is holonomic (the real AGV).
type BaseConfig = { label: string; cmdVelTopic: string; enableVy: boolean; note?: string };

// One Dex3 hand: preset + slider topics and the per-joint slider bounds (URDF limits).
type HandSide = {
  key: string;
  label: string;
  presetTopic: string;
  sliderTopic: string;
  joints: { name: string; min: number; max: number }[];
};

type RobotConfig = {
  label: string;
  arms: ArmGroup[];
  base?: BaseConfig;
  hands?: HandSide[];
};

const HAND_PRESETS = ["open", "close", "pinch"] as const;

// G1-D Dex3 finger joints in motor-index order, with URDF limits (mirrors
// fm_bringup/fm_bringup/hand_presets.py). Short labels keep the slider column readable.
function g1Hand(side: "left" | "right"): HandSide["joints"] {
  const L = side === "left";
  return [
    { name: `${side}_hand_thumb_0_joint`, min: -1.0472, max: 1.0472 },
    { name: `${side}_hand_thumb_1_joint`, min: L ? -0.7243 : -1.0472, max: L ? 1.0472 : 0.7243 },
    { name: `${side}_hand_thumb_2_joint`, min: L ? 0.0 : -1.7453, max: L ? 1.7453 : 0.0 },
    { name: `${side}_hand_middle_0_joint`, min: L ? -1.5708 : 0.0, max: L ? 0.0 : 1.5708 },
    { name: `${side}_hand_middle_1_joint`, min: L ? -1.7453 : 0.0, max: L ? 0.0 : 1.7453 },
    { name: `${side}_hand_index_0_joint`, min: L ? -1.5708 : 0.0, max: L ? 0.0 : 1.5708 },
    { name: `${side}_hand_index_1_joint`, min: L ? -1.7453 : 0.0, max: L ? 0.0 : 1.7453 },
  ];
}

const ROBOTS: Record<string, RobotConfig> = {
  openarm: {
    label: "OpenArm (right arm)",
    arms: [
      {
        key: "right",
        label: "Right arm",
        servoTopic: "/servo_node/delta_twist_cmds",
        jointTopic: "/servo_node/delta_joint_cmds",
        commandFrame: "openarm_right_base_link",
        joints: Array.from({ length: 7 }, (_, i) => `openarm_right_joint${i + 1}`),
        enableCartesian: true,
      },
    ],
  },
  // SO101: Servo drives the 5-joint manipulator; the gripper is a separate controller. A
  // 5-DOF arm cannot span SE(3), so Cartesian orientation drifts on the unobtainable axis.
  so101: {
    label: "SO101",
    arms: [
      {
        key: "arm",
        label: "Arm",
        servoTopic: "/servo_node/delta_twist_cmds",
        jointTopic: "/servo_node/delta_joint_cmds",
        commandFrame: "base_link",
        joints: ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
        enableCartesian: true,
        cartesianNote: "5-DOF: translation tracks, orientation drifts",
      },
    ],
  },
  // G1-D: both 7-DOF arms (each on its own servo_node, full 6-DOF Cartesian), the wheeled
  // base (sim diff-drive tracks vx + vyaw; the real AGV adds vy), and both Dex3 hands.
  // Arm-arm collision is not guarded — operator caution required.
  g1_d: {
    label: "G1-D (full body)",
    arms: [
      {
        key: "right",
        label: "Right arm",
        servoTopic: "/servo_node/delta_twist_cmds",
        jointTopic: "/servo_node/delta_joint_cmds",
        commandFrame: "torso_link",
        joints: [
          "right_shoulder_pitch_joint",
          "right_shoulder_roll_joint",
          "right_shoulder_yaw_joint",
          "right_elbow_joint",
          "right_wrist_roll_joint",
          "right_wrist_pitch_joint",
          "right_wrist_yaw_joint",
        ],
        enableCartesian: true,
      },
      {
        key: "left",
        label: "Left arm",
        servoTopic: "/servo_node_left/delta_twist_cmds",
        jointTopic: "/servo_node_left/delta_joint_cmds",
        commandFrame: "torso_link",
        joints: [
          "left_shoulder_pitch_joint",
          "left_shoulder_roll_joint",
          "left_shoulder_yaw_joint",
          "left_elbow_joint",
          "left_wrist_roll_joint",
          "left_wrist_pitch_joint",
          "left_wrist_yaw_joint",
        ],
        enableCartesian: true,
      },
    ],
    base: {
      label: "Base (vx · vyaw)",
      cmdVelTopic: "/cmd_vel",
      enableVy: false,
      note: "sim diff-drive: vx + vyaw",
    },
    hands: [
      {
        key: "left",
        label: "Left hand",
        presetTopic: "/g1_hand_teleop/left/preset",
        sliderTopic: "/g1_hand_teleop/left/sliders",
        joints: g1Hand("left"),
      },
      {
        key: "right",
        label: "Right hand",
        presetTopic: "/g1_hand_teleop/right/preset",
        sliderTopic: "/g1_hand_teleop/right/sliders",
        joints: g1Hand("right"),
      },
    ],
  },
};

const DEFAULT_ROBOT = "openarm";

type Axis = "linear" | "angular";
type Field = "x" | "y" | "z";
type Stamp = { sec: number; nsec: number };

function robotConfig(key: string): RobotConfig {
  return ROBOTS[key] ?? ROBOTS[DEFAULT_ROBOT]!;
}

// Every topic the current robot's panel can publish, with its schema, so the panel
// advertises exactly what it will use and tears them down on a robot switch.
function advertisedTopics(cfg: RobotConfig): Array<{ topic: string; schema: string }> {
  const out: Array<{ topic: string; schema: string }> = [];
  for (const arm of cfg.arms) {
    out.push({ topic: arm.servoTopic, schema: "geometry_msgs/msg/TwistStamped" });
    out.push({ topic: arm.jointTopic, schema: "control_msgs/msg/JointJog" });
  }
  if (cfg.base) {
    out.push({ topic: cfg.base.cmdVelTopic, schema: "geometry_msgs/msg/Twist" });
  }
  for (const hand of cfg.hands ?? []) {
    out.push({ topic: hand.presetTopic, schema: "std_msgs/msg/String" });
    out.push({ topic: hand.sliderTopic, schema: "std_msgs/msg/Float64MultiArray" });
  }
  return out;
}

function TeleopPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const initialRobot = (context.initialState as { robot?: string } | undefined)?.robot;
  const [robot, setRobot] = useState<string>(
    initialRobot && ROBOTS[initialRobot] ? initialRobot : DEFAULT_ROBOT,
  );
  const cfg = robotConfig(robot);
  // Active held command (arm twist/joint or base twist), refreshed by the repeat timer.
  const held = useRef<{ topic: string; make: (stamp: Stamp) => unknown } | undefined>();
  // Per-hand slider vectors (one entry per finger joint), keyed by hand.key.
  const [handValues, setHandValues] = useState<Record<string, number[]>>({});

  useLayoutEffect(() => {
    context.onRender = (_state, done) => setRenderDone(() => done);
    const topics = advertisedTopics(cfg);
    for (const { topic, schema } of topics) {
      context.advertise?.(topic, schema);
    }
    return () => {
      for (const { topic } of topics) {
        context.unadvertise?.(topic);
      }
    };
  }, [context, cfg]);

  // Reset slider state to each hand's neutral (open = 0) when the robot changes.
  useEffect(() => {
    const next: Record<string, number[]> = {};
    for (const hand of cfg.hands ?? []) {
      next[hand.key] = hand.joints.map(() => 0);
    }
    setHandValues(next);
    held.current = undefined;
  }, [cfg]);

  // Robot picker lives in the panel settings editor; persist the choice.
  useEffect(() => {
    const actionHandler = (action: SettingsTreeAction) => {
      if (action.action === "update" && action.payload.path[0] === "general" &&
          action.payload.path[1] === "robot") {
        const next = action.payload.value as string;
        held.current = undefined;
        setRobot(next);
        context.saveState({ robot: next });
      }
    };
    context.updatePanelSettingsEditor({
      actionHandler,
      nodes: {
        general: {
          label: "General",
          fields: {
            robot: {
              label: "Robot",
              input: "select",
              value: robot,
              options: Object.entries(ROBOTS).map(([key, c]) => ({ label: c.label, value: key })),
            },
          },
        },
      },
    });
  }, [context, robot]);

  // Re-publish the held command on a timer so motion continues while a button is pressed.
  useEffect(() => {
    const timer = setInterval(() => {
      const cmd = held.current;
      if (!cmd) return;
      context.publish?.(cmd.topic, cmd.make(nowStamp()));
    }, REPEAT_MS);
    return () => clearInterval(timer);
  }, [context]);

  useEffect(() => renderDone?.(), [renderDone]);

  const stop = () => {
    held.current = undefined;
  };

  const publishPreset = (hand: HandSide, name: string) => {
    context.publish?.(hand.presetTopic, { data: name });
  };

  const publishSlider = (hand: HandSide, index: number, value: number) => {
    setHandValues((prev) => {
      const current = prev[hand.key] ?? hand.joints.map(() => 0);
      const updated = current.slice();
      updated[index] = value;
      context.publish?.(hand.sliderTopic, {
        layout: { dim: [], data_offset: 0 },
        data: updated,
      });
      return { ...prev, [hand.key]: updated };
    });
  };

  return (
    <div style={{ padding: "0.75rem", fontFamily: "sans-serif" }}>
      <h3 style={{ marginTop: 0 }}>{cfg.label} Teleop</h3>

      {cfg.arms.map((arm) => (
        <div key={arm.key}>
          <h4 style={{ margin: "0.5rem 0 0.25rem" }}>{arm.label}</h4>
          {arm.enableCartesian && (
            <Section title={`Cartesian (m/s · rad/s, unitless)${arm.cartesianNote ? ` — ${arm.cartesianNote}` : ""}`}>
              {(["linear", "angular"] as Axis[]).map((axis) =>
                (["x", "y", "z"] as Field[]).map((field) => (
                  <JogButton
                    key={`${axis}-${field}`}
                    label={`${axis[0]}${field}`}
                    onDown={(sign) => {
                      held.current = {
                        topic: arm.servoTopic,
                        make: (stamp) => twistMsg(stamp, arm.commandFrame, axis, field, sign),
                      };
                    }}
                    onUp={stop}
                  />
                )),
              )}
            </Section>
          )}
          <Section title="Per-joint">
            {arm.joints.map((joint, i) => (
              <JogButton
                key={joint}
                label={`j${i + 1}`}
                onDown={(sign) => {
                  held.current = {
                    topic: arm.jointTopic,
                    make: (stamp) => jointMsg(stamp, arm.commandFrame, joint, sign),
                  };
                }}
                onUp={stop}
              />
            ))}
          </Section>
        </div>
      ))}

      {cfg.base && (
        <div>
          <h4 style={{ margin: "0.5rem 0 0.25rem" }}>{cfg.base.label}</h4>
          <Section title={cfg.base.note ?? "Base"}>
            <BaseJog field="x" axis="linear" label="vx" held={held} topic={cfg.base.cmdVelTopic} onUp={stop} />
            {cfg.base.enableVy && (
              <BaseJog field="y" axis="linear" label="vy" held={held} topic={cfg.base.cmdVelTopic} onUp={stop} />
            )}
            <BaseJog field="z" axis="angular" label="vyaw" held={held} topic={cfg.base.cmdVelTopic} onUp={stop} />
          </Section>
        </div>
      )}

      {(cfg.hands ?? []).map((hand) => (
        <div key={hand.key}>
          <h4 style={{ margin: "0.5rem 0 0.25rem" }}>{hand.label}</h4>
          <Section title="Presets">
            {HAND_PRESETS.map((name) => (
              <button key={name} onClick={() => publishPreset(hand, name)}>
                {name}
              </button>
            ))}
          </Section>
          <Section title="Sliders (rad)">
            {hand.joints.map((joint, i) => (
              <FingerSlider
                key={joint.name}
                label={shortJoint(joint.name)}
                min={joint.min}
                max={joint.max}
                value={handValues[hand.key]?.[i] ?? 0}
                onChange={(v) => publishSlider(hand, i, v)}
              />
            ))}
          </Section>
        </div>
      ))}
    </div>
  );
}

function nowStamp(): Stamp {
  const now = Date.now();
  return { sec: Math.floor(now / 1000), nsec: (now % 1000) * 1e6 };
}

function twistMsg(stamp: Stamp, frame: string, axis: Axis, field: Field, value: number) {
  const linear = { x: 0, y: 0, z: 0 };
  const angular = { x: 0, y: 0, z: 0 };
  (axis === "linear" ? linear : angular)[field] = value;
  return { header: { stamp, frame_id: frame }, twist: { linear, angular } };
}

function jointMsg(stamp: Stamp, frame: string, joint: string, value: number) {
  return {
    header: { stamp, frame_id: frame },
    joint_names: [joint],
    velocities: [value],
    displacements: [],
    duration: 0,
  };
}

// geometry_msgs/Twist for the wheeled base (no header). field selects vx / vy / vyaw.
function baseTwistMsg(axis: Axis, field: Field, value: number) {
  const linear = { x: 0, y: 0, z: 0 };
  const angular = { x: 0, y: 0, z: 0 };
  (axis === "linear" ? linear : angular)[field] = value;
  return { linear, angular };
}

function shortJoint(name: string): string {
  // left_hand_thumb_2_joint -> thumb_2
  return name.replace(/^(left|right)_hand_/, "").replace(/_joint$/, "");
}

function Section({ title, children }: { title: string; children: React.ReactNode }): ReactElement {
  return (
    <div style={{ marginBottom: "0.75rem" }}>
      <div style={{ fontSize: "0.8rem", opacity: 0.7, marginBottom: "0.25rem" }}>{title}</div>
      <div style={{ display: "flex", flexWrap: "wrap", gap: "0.25rem" }}>{children}</div>
    </div>
  );
}

// A pair of +/- buttons. Holding publishes a sustained command; release stops.
function JogButton({
  label,
  onDown,
  onUp,
}: {
  label: string;
  onDown: (sign: number) => void;
  onUp: () => void;
}): ReactElement {
  return (
    <div style={{ display: "flex", flexDirection: "column", alignItems: "center" }}>
      <button onPointerDown={() => onDown(1)} onPointerUp={onUp} onPointerLeave={onUp}>
        {label}+
      </button>
      <button onPointerDown={() => onDown(-1)} onPointerUp={onUp} onPointerLeave={onUp}>
        {label}-
      </button>
    </div>
  );
}

// Base jog button pair, wired to publish a geometry_msgs/Twist on the held timer.
function BaseJog({
  label,
  axis,
  field,
  topic,
  held,
  onUp,
}: {
  label: string;
  axis: Axis;
  field: Field;
  topic: string;
  held: React.MutableRefObject<{ topic: string; make: (stamp: Stamp) => unknown } | undefined>;
  onUp: () => void;
}): ReactElement {
  return (
    <JogButton
      label={label}
      onDown={(sign) => {
        held.current = { topic, make: () => baseTwistMsg(axis, field, sign) };
      }}
      onUp={onUp}
    />
  );
}

// A single finger slider; publishes the hand's full joint vector on change.
function FingerSlider({
  label,
  min,
  max,
  value,
  onChange,
}: {
  label: string;
  min: number;
  max: number;
  value: number;
  onChange: (value: number) => void;
}): ReactElement {
  return (
    <label style={{ display: "flex", alignItems: "center", gap: "0.25rem", width: "100%" }}>
      <span style={{ width: "5rem", fontSize: "0.75rem" }}>{label}</span>
      <input
        type="range"
        min={min}
        max={max}
        step={0.01}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        style={{ flex: 1 }}
      />
    </label>
  );
}

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "First Motive Teleop",
    initPanel: (context: PanelExtensionContext) => {
      const root = createRoot(context.panelElement);
      root.render(<TeleopPanel context={context} />);
      return () => root.unmount();
    },
  });
}
