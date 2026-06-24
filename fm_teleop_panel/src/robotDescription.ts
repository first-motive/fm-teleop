const KNOWN_ROBOTS = new Set(["openarm", "so101", "g1_d"]);

export function detectRobotKeyFromDescription(xml: string | undefined): string | undefined {
  if (!xml) {
    return undefined;
  }

  const match = xml.match(/<robot\b[^>]*\bname=["']([^"']+)["']/i);
  if (!match) {
    return undefined;
  }

  const name = match[1]?.trim();
  if (!name || !KNOWN_ROBOTS.has(name)) {
    return undefined;
  }
  return name;
}
