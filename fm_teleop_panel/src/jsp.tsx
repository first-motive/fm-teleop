// First Motive joint-state-publisher panel — a sibling to the teleop panel in
// the same extension.
//
// It reads /robot_description (a std_msgs/String URDF), draws one slider per
// movable joint bounded by the joint's limits, and publishes a
// sensor_msgs/JointState on /joint_command (topic set in panel settings) as the
// operator drags. On mount it seeds the sliders from the latest /joint_states so
// it opens at the robot's CURRENT pose (the home pose jsp published), never at
// model-zero — the "load at original positions" requirement.
//
// This is the Foxglove half of the single-publisher invariant. The launch runs
// joint_state_publisher as the SOLE /joint_states publisher, subscribed to
// /joint_command via source_list; this panel feeds that topic instead of
// publishing /joint_states itself, so the two never race.
//
// SIM PATH WARNING: never point this panel at a running sim. There
// joint_state_broadcaster owns /joint_states from the controllers; a second
// source fights them. This panel is for the description-view path only.

import { ExtensionContext, PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import { EMPTY_MODEL, JspModel, reconcile } from "./jsp-reconcile";
import { JointStateMessage } from "./jsp-seed";
import { MovableJoint } from "./urdf";

const DESCRIPTION_TOPIC = "/robot_description";
const JOINT_STATES_TOPIC = "/joint_states";
const DEFAULT_COMMAND_TOPIC = "/joint_command";
const JOINT_STATE_SCHEMA = "sensor_msgs/msg/JointState";

type Stamp = { sec: number; nsec: number };

// Persisted panel state (Foxglove saveState / initialState).
type PanelState = { topic?: string };

function nowStamp(): Stamp {
  const now = Date.now();
  return { sec: Math.floor(now / 1000), nsec: (now % 1000) * 1e6 };
}

function JointStatePanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const persisted = context.initialState as PanelState | undefined;
  const [topic, setTopic] = useState<string>(persisted?.topic ?? DEFAULT_COMMAND_TOPIC);

  const [joints, setJoints] = useState<MovableJoint[]>([]);
  const [values, setValues] = useState<Record<string, number>>({});

  // Incoming data + the reconcile model live in refs so onRender reconciles
  // without re-arming. reconcile (a pure step) owns the slider set + seeding.
  const latestState = useRef<JointStateMessage | undefined>(undefined);
  const latestUrdf = useRef<string | undefined>(undefined); // most recent /robot_description
  const model = useRef<JspModel>(EMPTY_MODEL);
  // topicRef mirrors topic so the publish handler reads the live value.
  const topicRef = useRef(topic);
  topicRef.current = topic;
  // jointsRef mirrors joints so the publish handler reads the current vector.
  const jointsRef = useRef<MovableJoint[]>([]);
  jointsRef.current = joints;

  useLayoutEffect(() => {
    context.watch("currentFrame");
    context.subscribe([{ topic: DESCRIPTION_TOPIC }, { topic: JOINT_STATES_TOPIC }]);

    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      for (const event of renderState.currentFrame ?? []) {
        if (event.topic === DESCRIPTION_TOPIC) {
          const urdf = (event.message as { data?: string }).data;
          if (typeof urdf === "string") {
            latestUrdf.current = urdf;
          }
        } else if (event.topic === JOINT_STATES_TOPIC) {
          latestState.current = event.message as JointStateMessage;
        }
      }

      const out = reconcile(model.current, latestUrdf.current, latestState.current);
      model.current = out.model;
      if (out.joints) {
        setJoints(out.joints);
      }
      if (out.seedValues) {
        setValues(out.seedValues);
      }
    };

    return () => {
      context.onRender = undefined;
    };
  }, [context]);

  // Advertise the command topic, re-advertising when the operator changes it.
  useEffect(() => {
    context.advertise?.(topic, JOINT_STATE_SCHEMA);
    return () => context.unadvertise?.(topic);
  }, [context, topic]);

  // Command topic lives in the panel settings editor; a change persists.
  useEffect(() => {
    const actionHandler = (action: SettingsTreeAction) => {
      if (
        action.action !== "update" ||
        action.payload.path[0] !== "general" ||
        action.payload.path[1] !== "topic"
      ) {
        return;
      }
      const next = action.payload.value as string;
      setTopic(next);
      context.saveState({ topic: next } satisfies PanelState);
    };
    context.updatePanelSettingsEditor({
      actionHandler,
      nodes: {
        general: {
          label: "General",
          fields: {
            topic: {
              label: "Command topic",
              input: "string",
              value: topic,
            },
          },
        },
      },
    });
  }, [context, topic]);

  useEffect(() => renderDone?.(), [renderDone]);

  // Publish the full joint vector on every slider move, stamped now, so jsp holds
  // a complete /joint_states (source_list republishes the last value it saw).
  const publish = (next: Record<string, number>) => {
    context.publish?.(topicRef.current, {
      header: { stamp: nowStamp(), frame_id: "" },
      name: jointsRef.current.map((j) => j.name),
      position: jointsRef.current.map((j) => next[j.name] ?? 0),
      velocity: [],
      effort: [],
    });
  };

  const onSlider = (name: string, value: number) => {
    setValues((prev) => {
      const next = { ...prev, [name]: value };
      publish(next);
      return next;
    });
  };

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
        gap: "0.5rem",
      }}
    >
      <header style={{ display: "flex", alignItems: "baseline", gap: "0.75rem" }}>
        <h3 style={{ margin: 0, flex: 1 }}>Joint State Publisher</h3>
        <span style={{ fontSize: "0.75rem", opacity: 0.7 }}>→ {topic}</span>
      </header>

      {joints.length === 0 ? (
        <p style={{ fontSize: "0.85rem", opacity: 0.7 }}>
          Waiting for {DESCRIPTION_TOPIC}… (start a robot view and enable the topic in the 3D
          panel).
        </p>
      ) : (
        <div style={{ display: "flex", flexDirection: "column", gap: "0.35rem" }}>
          {joints.map((joint) => (
            <JointSlider
              key={joint.name}
              joint={joint}
              value={values[joint.name] ?? 0}
              onChange={(v) => onSlider(joint.name, v)}
            />
          ))}
        </div>
      )}
    </div>
  );
}

// One labelled slider spanning a joint's limits, with a live radian readout.
function JointSlider({
  joint,
  value,
  onChange,
}: {
  joint: MovableJoint;
  value: number;
  onChange: (value: number) => void;
}): ReactElement {
  return (
    <label style={{ display: "flex", alignItems: "center", gap: "0.5rem", fontSize: "0.75rem" }}>
      <span style={{ width: "13rem", overflow: "hidden", textOverflow: "ellipsis" }}>
        {joint.name}
      </span>
      <input
        type="range"
        min={joint.lower}
        max={joint.upper}
        step={0.01}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        style={{ flex: 1 }}
      />
      <span style={{ width: "3.5rem", textAlign: "right", fontVariantNumeric: "tabular-nums" }}>
        {value.toFixed(2)}
      </span>
    </label>
  );
}

// Register the panel on the shared extension context. The teleop panel's
// activate() registers its own panel on the same context; both ship in one .foxe.
export function initJointStatePanel(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "First Motive Joint State Publisher",
    initPanel: (context: PanelExtensionContext) => {
      context.panelElement.style.overflowY = "auto";
      const root = createRoot(context.panelElement);
      root.render(<JointStatePanel context={context} />);
      return () => root.unmount();
    },
  });
}
