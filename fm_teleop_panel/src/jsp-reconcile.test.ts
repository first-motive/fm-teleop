import { describe, expect, it } from "vitest";

import { EMPTY_MODEL, reconcile } from "./jsp-reconcile";

const URDF = `<robot name="r">
  <joint name="a" type="revolute"><limit lower="-1" upper="1"/></joint>
  <joint name="b" type="revolute"><limit lower="0" upper="2"/></joint>
</robot>`;

describe("reconcile", () => {
  it("builds sliders from a new URDF, seeding zeros when no /joint_states yet", () => {
    const out = reconcile(EMPTY_MODEL, URDF, undefined);
    expect(out.joints?.map((j) => j.name)).toEqual(["a", "b"]);
    expect(out.seedValues).toEqual({ a: 0, b: 0 });
    expect(out.model.seeded).toBe(false); // no real state yet → still open to reseed
  });

  it("seeds from /joint_states when the URDF and state arrive together", () => {
    const out = reconcile(EMPTY_MODEL, URDF, { name: ["a", "b"], position: [0.5, 1.0] });
    expect(out.seedValues).toEqual({ a: 0.5, b: 1.0 });
    expect(out.model.seeded).toBe(true);
  });

  it("reseeds once when the first real /joint_states lands after the sliders", () => {
    const built = reconcile(EMPTY_MODEL, URDF, undefined).model; // sliders, zero-seeded
    const out = reconcile(built, URDF, { name: ["a", "b"], position: [0.3, 1.7] });
    expect(out.seedValues).toEqual({ a: 0.3, b: 1.7 });
    expect(out.model.seeded).toBe(true);
  });

  it("ignores later /joint_states once seeded (operator owns the sliders)", () => {
    const seeded = reconcile(EMPTY_MODEL, URDF, { name: ["a"], position: [0.5] }).model;
    const out = reconcile(seeded, URDF, { name: ["a"], position: [0.9] });
    expect(out.seedValues).toBeUndefined();
    expect(out.joints).toBeUndefined();
    expect(out.model).toBe(seeded); // unchanged reference → no React update
  });

  it("rebuilds and reseeds when the URDF changes (new robot loaded)", () => {
    const first = reconcile(EMPTY_MODEL, URDF, { name: ["a"], position: [0.5] }).model;
    const other = `<robot name="o"><joint name="z" type="revolute"><limit lower="0" upper="1"/></joint></robot>`;
    const out = reconcile(first, other, { name: ["z"], position: [0.4] });
    expect(out.joints?.map((j) => j.name)).toEqual(["z"]);
    expect(out.seedValues).toEqual({ z: 0.4 });
  });

  it("no-ops when nothing has arrived", () => {
    const out = reconcile(EMPTY_MODEL, undefined, undefined);
    expect(out).toEqual({ model: EMPTY_MODEL });
  });
});
