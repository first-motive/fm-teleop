// Extension entry point. The toolchain's default entry is src/index.ts; the
// panels (JSX) live in panel.tsx (teleop) and jsp.tsx (joint state publisher).
// One extension ships both panels — activate registers each on the shared
// context.
import { ExtensionContext } from "@foxglove/extension";

import { initJointStatePanel } from "./jsp";
import { activate as activateTeleop } from "./panel";

export function activate(extensionContext: ExtensionContext): void {
  activateTeleop(extensionContext);
  initJointStatePanel(extensionContext);
}
