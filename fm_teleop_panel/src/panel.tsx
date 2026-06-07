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

import { Joystick, ThumbStick } from "./joystick";
import { Contribution, mergeContributions, scaleContribution, toMessage, Vec3 } from "./merge";

const REPEAT_MS = 50;

// Default joystick centre deadzone (fraction of full deflection) for a fresh
// panel; the operator overrides it in the panel settings. Keeps resting drift out.
const DEFAULT_DEADZONE = 0.08;

const STICK_SIZE = 140; // primary joystick diameter, px
const THUMB_WIDTH = 48; // 1-axis thumb track width, px

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

type Stamp = { sec: number; nsec: number };

// Persisted panel state (Foxglove saveState / initialState).
type PanelState = { robot?: string; speed?: number; deadzone?: number };

function clamp01(v: number): number {
  return Math.max(0, Math.min(1, v));
}

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
  const persisted = context.initialState as PanelState | undefined;
  const [robot, setRobot] = useState<string>(
    persisted?.robot && ROBOTS[persisted.robot] ? persisted.robot : DEFAULT_ROBOT,
  );
  // Speed scalar (0..1) multiplies every published command magnitude; deadzone
  // is the joystick centre dead band. Both persist and are set in panel settings.
  const [speed, setSpeed] = useState<number>(clamp01(persisted?.speed ?? 1));
  const [deadzone, setDeadzone] = useState<number>(clamp01(persisted?.deadzone ?? DEFAULT_DEADZONE));
  // speedRef mirrors speed so the publish timer reads the live value without
  // re-arming the interval (its effect dep is [context] only). Deadzone needs no
  // such ref — it is passed as a prop and applied per pointer event by each stick.
  const speedRef = useRef(speed);
  speedRef.current = speed;
  const cfg = robotConfig(robot);
  // Active contributions keyed by widget id. Many widgets can be held at once
  // (two-thumb teleop); the repeat timer merges them per topic and publishes.
  const held = useRef<Map<string, Contribution>>(new Map());
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
    held.current.clear();
  }, [cfg]);

  // Robot picker, speed scalar, and deadzone live in the panel settings editor;
  // every change persists the full state so a reload restores it.
  useEffect(() => {
    const actionHandler = (action: SettingsTreeAction) => {
      if (action.action !== "update" || action.payload.path[0] !== "general") {
        return;
      }
      const field = action.payload.path[1];
      if (field === "robot") {
        const next = action.payload.value as string;
        held.current.clear();
        setRobot(next);
        context.saveState({ robot: next, speed, deadzone } satisfies PanelState);
      } else if (field === "speed") {
        const next = clamp01(action.payload.value as number);
        setSpeed(next);
        context.saveState({ robot, speed: next, deadzone } satisfies PanelState);
      } else if (field === "deadzone") {
        const next = clamp01(action.payload.value as number);
        setDeadzone(next);
        context.saveState({ robot, speed, deadzone: next } satisfies PanelState);
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
            speed: {
              label: "Speed scalar",
              input: "number",
              value: speed,
              min: 0,
              max: 1,
              step: 0.05,
            },
            deadzone: {
              label: "Joystick deadzone",
              input: "number",
              value: deadzone,
              min: 0,
              max: 0.9,
              step: 0.01,
            },
          },
        },
      },
    });
  }, [context, robot, speed, deadzone]);

  // Re-publish held commands on a timer so motion continues while widgets are held.
  // All active contributions merge per topic, so two-thumb drags publish together.
  useEffect(() => {
    const timer = setInterval(() => {
      const active = held.current;
      if (active.size === 0) return;
      const stamp = nowStamp();
      const factor = speedRef.current;
      for (const c of mergeContributions(active.values())) {
        context.publish?.(c.topic, toMessage(scaleContribution(c, factor), stamp));
      }
    }, REPEAT_MS);
    return () => clearInterval(timer);
  }, [context]);

  useEffect(() => renderDone?.(), [renderDone]);

  // Widgets register their contribution by a stable id on press, and clear it on
  // release. stop() drops every active contribution at once (the global STOP).
  const setHeld = (id: string, c: Contribution) => {
    held.current.set(id, c);
  };
  const clearHeld = (id: string) => {
    held.current.delete(id);
  };
  const stop = () => {
    held.current.clear();
  };

  // Speed slider in the dashboard header mirrors the settings field; both persist.
  const onSpeedChange = (v: number) => {
    const next = clamp01(v);
    setSpeed(next);
    context.saveState({ robot, speed: next, deadzone } satisfies PanelState);
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

  const hands = cfg.hands ?? [];

  return (
    <div
      style={{
        height: "100%",
        overflowY: "auto",
        boxSizing: "border-box",
        padding: "0.75rem",
        fontFamily: "sans-serif",
        display: "flex",
        flexDirection: "column",
        gap: "0.75rem",
      }}
    >
      {/* Dashboard header: title, always-reachable global STOP, speed scalar. */}
      <header style={{ display: "flex", alignItems: "center", gap: "1rem", flexWrap: "wrap" }}>
        <h3 style={{ margin: 0, flex: 1 }}>{cfg.label} Teleop</h3>
        <button
          onClick={stop}
          style={{
            background: "#c0392b",
            color: "#fff",
            border: "none",
            borderRadius: "0.25rem",
            padding: "0.5rem 1.25rem",
            fontWeight: "bold",
            cursor: "pointer",
          }}
        >
          STOP
        </button>
        <label style={{ display: "flex", alignItems: "center", gap: "0.4rem", fontSize: "0.8rem" }}>
          Speed
          <input
            type="range"
            min={0}
            max={1}
            step={0.05}
            value={speed}
            onChange={(e) => onSpeedChange(parseFloat(e.target.value))}
          />
          <span style={{ width: "2.5rem", textAlign: "right" }}>{Math.round(speed * 100)}%</span>
        </label>
      </header>

      {/* Primary controls: arm Cartesian sticks + base, always on, in a grid that
          reflows from two-up on a tablet to one column on a phone. */}
      <div
        style={{
          display: "grid",
          gridTemplateColumns: "repeat(auto-fit, minmax(320px, 1fr))",
          gap: "0.75rem",
        }}
      >
        {cfg.arms.map((arm) => (
          <section key={arm.key}>
            <h4 style={{ margin: "0 0 0.25rem" }}>{arm.label}</h4>
            {arm.enableCartesian && (
              <Section title={`Cartesian (unitless)${arm.cartesianNote ? ` — ${arm.cartesianNote}` : ""}`}>
                <ArmCartesianSticks arm={arm} deadzone={deadzone} setHeld={setHeld} clearHeld={clearHeld} />
              </Section>
            )}
            <details>
              <summary style={{ fontSize: "0.8rem", opacity: 0.7, cursor: "pointer" }}>Per-joint</summary>
              <div style={{ display: "flex", flexWrap: "wrap", gap: "0.25rem", marginTop: "0.25rem" }}>
                {arm.joints.map((joint, i) => {
                  const id = `${arm.key}-joint-${joint}`;
                  return (
                    <JogButton
                      key={joint}
                      label={`j${i + 1}`}
                      onDown={(sign) =>
                        setHeld(id, {
                          kind: "jointJog",
                          topic: arm.jointTopic,
                          frame: arm.commandFrame,
                          velocities: { [joint]: sign },
                        })
                      }
                      onUp={() => clearHeld(id)}
                    />
                  );
                })}
              </div>
            </details>
          </section>
        ))}

        {cfg.base && (
          <section>
            <h4 style={{ margin: "0 0 0.25rem" }}>{cfg.base.label}</h4>
            <Section title={cfg.base.note ?? "Base"}>
              <BaseJoystick base={cfg.base} deadzone={deadzone} setHeld={setHeld} clearHeld={clearHeld} />
            </Section>
          </section>
        )}
      </div>

      {/* Secondary controls: hand presets + per-finger sliders, collapsed by default. */}
      {hands.length > 0 && (
        <details>
          <summary style={{ fontSize: "0.85rem", cursor: "pointer" }}>Hands</summary>
          <div
            style={{
              display: "grid",
              gridTemplateColumns: "repeat(auto-fit, minmax(280px, 1fr))",
              gap: "0.75rem",
              marginTop: "0.5rem",
            }}
          >
            {hands.map((hand) => (
              <div key={hand.key}>
                <h4 style={{ margin: "0 0 0.25rem" }}>{hand.label}</h4>
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
        </details>
      )}
    </div>
  );
}

function nowStamp(): Stamp {
  const now = Date.now();
  return { sec: Math.floor(now / 1000), nsec: (now % 1000) * 1e6 };
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

// Base drive on one joystick: push up for forward (vx), push sideways to turn
// (vyaw). A holonomic base adds a vertical thumb for lateral strafe (vy). Each
// control writes a geometry_msgs/Twist contribution; the timer merges and
// publishes them on /cmd_vel. A control at rest clears its contribution so the
// base command times out to a stop.
function BaseJoystick({
  base,
  deadzone,
  setHeld,
  clearHeld,
}: {
  base: BaseConfig;
  deadzone: number;
  setHeld: (id: string, c: Contribution) => void;
  clearHeld: (id: string) => void;
}): ReactElement {
  const topic = base.cmdVelTopic;
  return (
    <div style={{ display: "flex", gap: "1rem", alignItems: "center", flexWrap: "wrap" }}>
      <Joystick
        size={STICK_SIZE}
        deadzone={deadzone}
        label="vx ↑ · vyaw ↔"
        onChange={(v) => {
          if (v.x === 0 && v.y === 0) {
            clearHeld("base-drive");
            return;
          }
          // Stick up (v.y > 0) drives forward; stick right (v.x > 0) yaws right,
          // which is negative angular.z under REP-103 (z up, +yaw is CCW/left).
          setHeld("base-drive", {
            kind: "twist",
            topic,
            linear: { x: v.y, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: -v.x },
          });
        }}
      />
      {base.enableVy && (
        <ThumbStick
          width={THUMB_WIDTH}
          height={STICK_SIZE}
          deadzone={deadzone}
          label="vy"
          onChange={(value) => {
            if (value === 0) {
              clearHeld("base-strafe");
              return;
            }
            setHeld("base-strafe", {
              kind: "twist",
              topic,
              linear: { x: 0, y: value, z: 0 },
              angular: { x: 0, y: 0, z: 0 },
            });
          }}
        />
      )}
    </div>
  );
}

// Cartesian arm jog on four sticks, all merged into one TwistStamped on the
// arm's servo topic (the held Map sums them per tick — full 6-DOF two-handed):
//   translate pad   lin x (fwd, up) · lin y (lateral, sideways)
//   Z thumb         lin z (up/down)
//   rotate pad      ang y (pitch, up/down) · ang z (yaw, sideways)
//   roll thumb      ang x (roll)
// Each stick clears its contribution at rest so Servo times the jog out to a hold.
function ArmCartesianSticks({
  arm,
  deadzone,
  setHeld,
  clearHeld,
}: {
  arm: ArmGroup;
  deadzone: number;
  setHeld: (id: string, c: Contribution) => void;
  clearHeld: (id: string) => void;
}): ReactElement {
  const topic = arm.servoTopic;
  const frame = arm.commandFrame;
  const write = (id: string, linear: Vec3, angular: Vec3) => {
    setHeld(id, { kind: "twistStamped", topic, frame, linear, angular });
  };
  const translateId = `${arm.key}-translate`;
  const rotateId = `${arm.key}-rotate`;
  const zId = `${arm.key}-zlift`;
  const rollId = `${arm.key}-roll`;

  return (
    <div style={{ display: "flex", gap: "1rem", flexWrap: "wrap" }}>
      <div style={{ display: "flex", gap: "0.5rem", alignItems: "center" }}>
        <Joystick
          size={STICK_SIZE}
          deadzone={deadzone}
          label="translate (x↑ · y↔)"
          onChange={(v) => {
            if (v.x === 0 && v.y === 0) {
              clearHeld(translateId);
              return;
            }
            // Stick up → forward (+x); stick right → robot-right (−y, REP y-left).
            write(translateId, { x: v.y, y: -v.x, z: 0 }, { x: 0, y: 0, z: 0 });
          }}
        />
        <ThumbStick
          width={THUMB_WIDTH}
          height={STICK_SIZE}
          deadzone={deadzone}
          label="Z"
          onChange={(value) => {
            if (value === 0) {
              clearHeld(zId);
              return;
            }
            write(zId, { x: 0, y: 0, z: value }, { x: 0, y: 0, z: 0 });
          }}
        />
      </div>
      <div style={{ display: "flex", gap: "0.5rem", alignItems: "center" }}>
        <Joystick
          size={STICK_SIZE}
          deadzone={deadzone}
          label="rotate (pitch↕ · yaw↔)"
          onChange={(v) => {
            if (v.x === 0 && v.y === 0) {
              clearHeld(rotateId);
              return;
            }
            // Stick up → pitch (+ang.y); stick right → yaw-right (−ang.z, REP z-up).
            write(rotateId, { x: 0, y: 0, z: 0 }, { x: 0, y: v.y, z: -v.x });
          }}
        />
        <ThumbStick
          width={THUMB_WIDTH}
          height={STICK_SIZE}
          deadzone={deadzone}
          label="roll"
          onChange={(value) => {
            if (value === 0) {
              clearHeld(rollId);
              return;
            }
            write(rollId, { x: 0, y: 0, z: 0 }, { x: value, y: 0, z: 0 });
          }}
        />
      </div>
    </div>
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
      // Foxglove sizes panelElement to fill the panel tile; make it the scroll
      // boundary so a tall dashboard scrolls instead of clipping off the bottom.
      context.panelElement.style.overflowY = "auto";
      const root = createRoot(context.panelElement);
      root.render(<TeleopPanel context={context} />);
      return () => root.unmount();
    },
  });
}
