// Hand-rolled joystick widgets — zero dependencies, SVG + pointer events.
//
// `Joystick` is a 2-axis pad; `ThumbStick` is a 1-axis vertical slider for a
// single DOF (Z lift, roll). Both capture the pointer on press so a drag that
// leaves the element still tracks, clamp the knob to the rim, normalize through
// joystick-math, and spring back to centre on release (emitting a zero).
//
// Each widget owns only its visual knob position; the command vector it emits
// via onChange is the caller's to route to a topic. Multi-touch falls out of
// pointer capture: two widgets dragged at once each own their own pointer.

import { PointerEvent as ReactPointerEvent, ReactElement, useRef, useState } from "react";

import { clampToRadius, stickVector, Vec2 } from "./joystick-math";

const KNOB_FRACTION = 0.32; // knob radius as a fraction of the pad radius

type JoystickProps = {
  size: number;
  deadzone: number;
  onChange: (v: Vec2) => void;
  label?: string;
};

// 2-axis pad. Emits a normalized vector (x right, y up) within the unit disk.
export function Joystick({ size, deadzone, onChange, label }: JoystickProps): ReactElement {
  const radius = size / 2;
  const knobR = radius * KNOB_FRACTION;
  const live = radius - knobR; // travel left for the knob centre
  const ref = useRef<SVGSVGElement>(null);
  const [knob, setKnob] = useState<Vec2>({ x: 0, y: 0 });
  const [out, setOut] = useState<Vec2>({ x: 0, y: 0 }); // live readout

  const handle = (e: ReactPointerEvent<SVGSVGElement>) => {
    const svg = ref.current;
    if (!svg) return;
    const rect = svg.getBoundingClientRect();
    const dx = e.clientX - (rect.left + radius);
    const dy = e.clientY - (rect.top + radius);
    setKnob(clampToRadius(dx, dy, live)); // clamp the visual knob to the rim
    const v = stickVector(dx, dy, live, deadzone);
    setOut(v);
    onChange(v);
  };

  const release = (e: ReactPointerEvent<SVGSVGElement>) => {
    e.currentTarget.releasePointerCapture?.(e.pointerId);
    setKnob({ x: 0, y: 0 });
    setOut({ x: 0, y: 0 });
    onChange({ x: 0, y: 0 });
  };

  return (
    <figure style={{ margin: 0, textAlign: "center" }}>
      <svg
        ref={ref}
        width={size}
        height={size}
        style={{ touchAction: "none", cursor: "pointer" }}
        onPointerDown={(e) => {
          e.currentTarget.setPointerCapture?.(e.pointerId);
          handle(e);
        }}
        onPointerMove={(e) => {
          if (e.buttons === 0) return;
          handle(e);
        }}
        onPointerUp={release}
        onPointerCancel={release}
      >
        <circle cx={radius} cy={radius} r={radius - 1} fill="#1e1e1e" stroke="#555" />
        <line x1={radius} y1={radius - live} x2={radius} y2={radius + live} stroke="#333" />
        <line x1={radius - live} y1={radius} x2={radius + live} y2={radius} stroke="#333" />
        <circle cx={radius + knob.x} cy={radius + knob.y} r={knobR} fill="#4a90d9" stroke="#7ab" />
      </svg>
      {label && <figcaption style={{ fontSize: "0.7rem", opacity: 0.7 }}>{label}</figcaption>}
      <Readout text={`${fmt(out.x)} · ${fmt(out.y)}`} />
    </figure>
  );
}

type ThumbStickProps = {
  width: number;
  height: number;
  deadzone: number;
  onChange: (value: number) => void;
  label?: string;
};

// 1-axis vertical slider. Emits a single scalar in [-1, 1], up-positive.
export function ThumbStick({ width, height, deadzone, onChange, label }: ThumbStickProps): ReactElement {
  const knobR = width / 2 - 1;
  const live = height / 2 - knobR; // vertical travel for the knob centre
  const ref = useRef<SVGSVGElement>(null);
  const [knobY, setKnobY] = useState(0);
  const [out, setOut] = useState(0); // live readout

  const handle = (e: ReactPointerEvent<SVGSVGElement>) => {
    const svg = ref.current;
    if (!svg) return;
    const rect = svg.getBoundingClientRect();
    const dy = e.clientY - (rect.top + height / 2);
    const clamped = Math.max(-live, Math.min(live, dy));
    setKnobY(clamped);
    const value = stickVector(0, dy, live, deadzone).y;
    setOut(value);
    onChange(value);
  };

  const release = (e: ReactPointerEvent<SVGSVGElement>) => {
    e.currentTarget.releasePointerCapture?.(e.pointerId);
    setKnobY(0);
    setOut(0);
    onChange(0);
  };

  return (
    <figure style={{ margin: 0, textAlign: "center" }}>
      <svg
        ref={ref}
        width={width}
        height={height}
        style={{ touchAction: "none", cursor: "pointer" }}
        onPointerDown={(e) => {
          e.currentTarget.setPointerCapture?.(e.pointerId);
          handle(e);
        }}
        onPointerMove={(e) => {
          if (e.buttons === 0) return;
          handle(e);
        }}
        onPointerUp={release}
        onPointerCancel={release}
      >
        <rect
          x={1}
          y={1}
          width={width - 2}
          height={height - 2}
          rx={width / 2}
          fill="#1e1e1e"
          stroke="#555"
        />
        <circle cx={width / 2} cy={height / 2 + knobY} r={knobR} fill="#4a90d9" stroke="#7ab" />
      </svg>
      {label && <figcaption style={{ fontSize: "0.7rem", opacity: 0.7 }}>{label}</figcaption>}
      <Readout text={fmt(out)} />
    </figure>
  );
}

// Signed, fixed-width value for the live readout (e.g. "+0.42", "−1.00").
function fmt(v: number): string {
  const s = v.toFixed(2);
  return v < 0 ? s.replace("-", "−") : `+${s}`;
}

function Readout({ text }: { text: string }): ReactElement {
  return (
    <div style={{ fontSize: "0.65rem", opacity: 0.6, fontVariantNumeric: "tabular-nums" }}>{text}</div>
  );
}
