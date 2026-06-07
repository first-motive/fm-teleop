import { describe, expect, it } from "vitest";
import { applyDeadzone, clampToRadius, stickVector } from "./joystick-math";

describe("clampToRadius", () => {
  it("leaves an offset inside the radius untouched", () => {
    expect(clampToRadius(3, 4, 10)).toEqual({ x: 3, y: 4 });
  });

  it("clamps an offset past the rim onto the rim, preserving direction", () => {
    const c = clampToRadius(30, 40, 10); // magnitude 50, radius 10
    expect(Math.hypot(c.x, c.y)).toBeCloseTo(10);
    expect(c.x / c.y).toBeCloseTo(30 / 40); // direction held
  });

  it("returns the centre for a zero offset", () => {
    expect(clampToRadius(0, 0, 10)).toEqual({ x: 0, y: 0 });
  });
});

describe("applyDeadzone", () => {
  it("returns zero inside the deadzone", () => {
    expect(applyDeadzone({ x: 0.1, y: 0 }, 0.2)).toEqual({ x: 0, y: 0 });
  });

  it("returns zero exactly at the centre", () => {
    expect(applyDeadzone({ x: 0, y: 0 }, 0.2)).toEqual({ x: 0, y: 0 });
  });

  it("rescales the live range so full deflection still reaches 1", () => {
    const v = applyDeadzone({ x: 1, y: 0 }, 0.2);
    expect(v.x).toBeCloseTo(1);
  });

  it("rescales a mid-range magnitude correctly", () => {
    // m=0.6, dz=0.2 -> (0.6-0.2)/(1-0.2) = 0.5
    const v = applyDeadzone({ x: 0.6, y: 0 }, 0.2);
    expect(v.x).toBeCloseTo(0.5);
  });

  it("preserves direction while rescaling", () => {
    const v = applyDeadzone({ x: 0.6, y: 0.8 }, 0.5); // magnitude 1.0
    expect(v.x / v.y).toBeCloseTo(0.6 / 0.8);
  });
});

describe("stickVector", () => {
  it("flips screen-down to command-up (drag up is positive y)", () => {
    const v = stickVector(0, -10, 10, 0); // pointer 10px above centre
    expect(v.y).toBeCloseTo(1);
    expect(v.x).toBeCloseTo(0);
  });

  it("drag down is negative y", () => {
    const v = stickVector(0, 10, 10, 0);
    expect(v.y).toBeCloseTo(-1);
  });

  it("drag right is positive x", () => {
    const v = stickVector(10, 0, 10, 0);
    expect(v.x).toBeCloseTo(1);
  });

  it("keeps the result within the unit disk past the rim", () => {
    const v = stickVector(100, 100, 10, 0);
    expect(Math.hypot(v.x, v.y)).toBeLessThanOrEqual(1.0001);
  });

  it("applies the deadzone at the centre", () => {
    const v = stickVector(1, 0, 10, 0.2); // normalized 0.1, below dz
    expect(v).toEqual({ x: 0, y: 0 });
  });

  it("returns zero for a non-positive radius", () => {
    expect(stickVector(5, 5, 0, 0)).toEqual({ x: 0, y: 0 });
  });
});
