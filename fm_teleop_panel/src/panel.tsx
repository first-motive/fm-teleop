// First Motive teleop panel — robot-aware.
//
// Publishes the two command streams MoveIt Servo consumes:
//   geometry_msgs/TwistStamped -> /servo_node/delta_twist_cmds  (Cartesian jog)
//   control_msgs/JointJog      -> /servo_node/delta_joint_cmds  (per-joint jog)
//
// Commands are unitless ([-1, 1]); Servo scales them (servo.yaml). Buttons send a
// short burst while held via a repeat timer, matching Servo's incoming_command_timeout.
//
// The joint set, command frame, and whether Cartesian jogging is offered are read
// from a per-robot config (ROBOTS below) selected in the panel settings, mirroring
// fm_bringup's robot registry. Adding a robot is one ROBOTS entry. The Servo command
// topics are fixed (one servo_node per running teleop), so they stay module-level.
//
// This is the scalable teleop spine: a new operator opens a Foxglove URL — no
// per-operator hardware. Build + install with the scripts in package.json (needs
// Node + the create-foxglove-extension toolchain); not built by the ROS workspace.

import { ExtensionContext, PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

const TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const REPEAT_MS = 50;

// Per-robot teleop surface. `commandFrame` must match servo.yaml's
// robot_link_command_frame for that robot; `joints` must match the Servo group's
// joints in order. `enableCartesian` hides the Cartesian section for arms that
// only jog per-joint; `cartesianNote` flags reduced-DOF caveats.
type RobotConfig = {
  label: string;
  commandFrame: string;
  joints: string[];
  enableCartesian: boolean;
  cartesianNote?: string;
};

const ROBOTS: Record<string, RobotConfig> = {
  openarm: {
    label: "OpenArm (right arm)",
    commandFrame: "openarm_right_base_link",
    joints: Array.from({ length: 7 }, (_, i) => `openarm_right_joint${i + 1}`),
    enableCartesian: true,
  },
  // SO101: Servo drives the 5-joint manipulator group; the gripper is a separate
  // controller, not jogged here. Cartesian runs through the inverse Jacobian — a
  // 5-DOF arm cannot span SE(3), so orientation drifts on the unobtainable axis.
  so101: {
    label: "SO101",
    commandFrame: "base_link",
    joints: ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
    enableCartesian: true,
    cartesianNote: "5-DOF: translation tracks, orientation drifts",
  },
  // G1-D: Servo drives the 7-joint right_arm group (waist/legs/left arm hold). The
  // 7-DOF arm spans SE(3), so full 6-DOF Cartesian works. The wheeled base is driven
  // separately (Twist -> AGV), not from this panel.
  g1_d: {
    label: "G1-D (right arm)",
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
};

const DEFAULT_ROBOT = "openarm";

type Axis = "linear" | "angular";

function robotConfig(key: string): RobotConfig {
  return ROBOTS[key] ?? ROBOTS[DEFAULT_ROBOT]!;
}

function TeleopPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const initialRobot = (context.initialState as { robot?: string } | undefined)?.robot;
  const [robot, setRobot] = useState<string>(
    initialRobot && ROBOTS[initialRobot] ? initialRobot : DEFAULT_ROBOT,
  );
  const cfg = robotConfig(robot);
  // Active command refreshed by the repeat timer while a button is held.
  const held = useRef<{ kind: "twist" | "joint"; payload: unknown } | undefined>();

  useLayoutEffect(() => {
    context.onRender = (_state, done) => setRenderDone(() => done);
    context.advertise?.(TWIST_TOPIC, "geometry_msgs/msg/TwistStamped");
    context.advertise?.(JOINT_TOPIC, "control_msgs/msg/JointJog");
    return () => {
      context.unadvertise?.(TWIST_TOPIC);
      context.unadvertise?.(JOINT_TOPIC);
    };
  }, [context]);

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

  // Re-publish the held command on a timer so Servo keeps moving while pressed.
  useEffect(() => {
    const timer = setInterval(() => {
      const cmd = held.current;
      if (!cmd) return;
      const stamp = nowStamp();
      if (cmd.kind === "twist") {
        const { axis, field, value } = cmd.payload as TwistCmd;
        context.publish?.(TWIST_TOPIC, twistMsg(stamp, cfg.commandFrame, axis, field, value));
      } else {
        const { joint, value } = cmd.payload as JointCmd;
        context.publish?.(JOINT_TOPIC, jointMsg(stamp, cfg.commandFrame, joint, value));
      }
    }, REPEAT_MS);
    return () => clearInterval(timer);
  }, [context, cfg]);

  useEffect(() => renderDone?.(), [renderDone]);

  const start = (cmd: { kind: "twist" | "joint"; payload: unknown }) => {
    held.current = cmd;
  };
  const stop = () => {
    held.current = undefined;
  };

  return (
    <div style={{ padding: "0.75rem", fontFamily: "sans-serif" }}>
      <h3 style={{ marginTop: 0 }}>{cfg.label} Teleop → Servo</h3>
      {cfg.enableCartesian && (
        <Section title={`Cartesian (m/s · rad/s, unitless)${cfg.cartesianNote ? ` — ${cfg.cartesianNote}` : ""}`}>
          {(["linear", "angular"] as Axis[]).map((axis) =>
            (["x", "y", "z"] as const).map((field) => (
              <JogButton
                key={`${axis}-${field}`}
                label={`${axis[0]}${field}`}
                onDown={(sign) => start({ kind: "twist", payload: { axis, field, value: sign } })}
                onUp={stop}
              />
            )),
          )}
        </Section>
      )}
      <Section title="Per-joint">
        {cfg.joints.map((joint, i) => (
          <JogButton
            key={joint}
            label={`j${i + 1}`}
            onDown={(sign) => start({ kind: "joint", payload: { joint, value: sign } })}
            onUp={stop}
          />
        ))}
      </Section>
    </div>
  );
}

type TwistCmd = { axis: Axis; field: "x" | "y" | "z"; value: number };
type JointCmd = { joint: string; value: number };

function nowStamp() {
  const now = Date.now();
  return { sec: Math.floor(now / 1000), nsec: (now % 1000) * 1e6 };
}

function twistMsg(
  stamp: { sec: number; nsec: number },
  frame: string,
  axis: Axis,
  field: string,
  value: number,
) {
  const linear = { x: 0, y: 0, z: 0 };
  const angular = { x: 0, y: 0, z: 0 };
  (axis === "linear" ? linear : angular)[field as "x" | "y" | "z"] = value;
  return { header: { stamp, frame_id: frame }, twist: { linear, angular } };
}

function jointMsg(
  stamp: { sec: number; nsec: number },
  frame: string,
  joint: string,
  value: number,
) {
  return {
    header: { stamp, frame_id: frame },
    joint_names: [joint],
    velocities: [value],
    displacements: [],
    duration: 0,
  };
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
