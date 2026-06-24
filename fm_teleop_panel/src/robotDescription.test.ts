import { describe, expect, it } from "vitest";

import { detectRobotKeyFromDescription } from "./robotDescription";

describe("detectRobotKeyFromDescription", () => {
  it("extracts a known robot name from URDF", () => {
    expect(detectRobotKeyFromDescription('<robot name="so101"></robot>')).toBe("so101");
    expect(detectRobotKeyFromDescription("<robot name='openarm'></robot>")).toBe("openarm");
  });

  it("ignores unknown robot names", () => {
    expect(detectRobotKeyFromDescription('<robot name="mystery_bot"></robot>')).toBeUndefined();
  });

  it("returns undefined for missing or malformed xml", () => {
    expect(detectRobotKeyFromDescription(undefined)).toBeUndefined();
    expect(detectRobotKeyFromDescription("")).toBeUndefined();
    expect(detectRobotKeyFromDescription("<robot></robot>")).toBeUndefined();
  });
});
