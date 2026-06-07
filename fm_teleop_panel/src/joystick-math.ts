// Pure geometry for the hand-rolled joystick: turn a pointer offset from the
// stick centre into a normalized, deadzoned command vector. No DOM, no React —
// so it is unit-testable in isolation (joystick-math.test.ts).
//
// Coordinate convention: input dx/dy are pixels from centre in screen space
// (x right, y DOWN). Output is normalized to the unit disk in robot/command
// space (x right, y UP) — screen-down is flipped so dragging the knob up gives a
// positive y, matching "up = forward / +vx".

export type Vec2 = { x: number; y: number };

function magnitude(v: Vec2): number {
  return Math.hypot(v.x, v.y);
}

// Clamp a pointer offset (px) to the stick radius, preserving direction. A pull
// past the rim sits on the rim; inside the rim is untouched.
export function clampToRadius(dx: number, dy: number, radius: number): Vec2 {
  const m = Math.hypot(dx, dy);
  if (m <= radius || m === 0) {
    return { x: dx, y: dy };
  }
  const scale = radius / m;
  return { x: dx * scale, y: dy * scale };
}

// Radial deadzone: a normalized vector (magnitude 0..1) whose magnitude is below
// `deadzone` returns the zero vector; above, the magnitude is rescaled so the
// output still spans 0..1 across the live range. Direction is preserved.
export function applyDeadzone(v: Vec2, deadzone: number): Vec2 {
  const m = magnitude(v);
  if (m <= deadzone || m === 0) {
    return { x: 0, y: 0 };
  }
  if (deadzone >= 1) {
    return { x: 0, y: 0 };
  }
  const scaled = (m - deadzone) / (1 - deadzone);
  const factor = scaled / m;
  return { x: v.x * factor, y: v.y * factor };
}

// Full transform: pointer offset (px from centre) -> normalized [-1, 1] command
// vector with y flipped to up-positive and the deadzone applied. The result
// sits within the unit disk, so |result| <= 1.
export function stickVector(dx: number, dy: number, radius: number, deadzone: number): Vec2 {
  if (radius <= 0) {
    return { x: 0, y: 0 };
  }
  const clamped = clampToRadius(dx, dy, radius);
  const normalized = { x: clamped.x / radius, y: -clamped.y / radius };
  return applyDeadzone(normalized, deadzone);
}
