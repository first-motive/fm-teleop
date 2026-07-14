#!/usr/bin/env python3
"""export_episode — the data engine CLI: a captured mirror session -> a training-ready episode.

Reads a session dir (mirror_log.jsonl + hands.jsonl + meta.json), joins the human hand
skeletons to the robot trajectory on the shared tick clock, and writes:

    episode.jsonl   one aligned record per timestep (human hands + robot state + action)
    episode.json    the feature schema + metadata (frames, fps, hand coverage, hand_qa grade)

Run offline (no ROS graph needed):

    ros2 run fm_teleop_vision export_episode <session_dir> [--out DIR] [--format jsonl|parquet]

or directly:  python3 -m fm_teleop_vision.export_episode <session_dir>
"""

import argparse
import os

from fm_teleop_vision import episode


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session_dir", help="capture session dir (has mirror_log.jsonl + hands.jsonl)")
    ap.add_argument("--out", default=None, help="output dir (default: the session dir)")
    ap.add_argument("--format", choices=("jsonl", "parquet"), default="jsonl",
                    help="record file format (parquet needs pyarrow)")
    ap.add_argument("--no-ee", action="store_true", help="omit the EE pose from observation.state")
    args = ap.parse_args()

    session = episode.load_session(args.session_dir)
    if not session["log"]:
        raise SystemExit("no mirror_log.jsonl rows in %s — nothing to export" % args.session_dir)
    ep = episode.build_episode(session, include_ee=not args.no_ee)

    out_dir = args.out or args.session_dir
    os.makedirs(out_dir, exist_ok=True)
    meta_path = os.path.join(out_dir, "episode.json")
    episode.write_meta(ep, meta_path)
    if args.format == "parquet":
        rec_path = os.path.join(out_dir, "episode.parquet")
        episode.write_parquet(ep, rec_path)
    else:
        rec_path = os.path.join(out_dir, "episode.jsonl")
        episode.write_jsonl(ep, rec_path)

    m = ep["meta"]
    cov = m["hand_coverage"]
    qa = (m.get("hand_qa") or {})
    print("episode: %s frames, %.1fs @ %s fps" % (m["frames"], m["duration_s"], m["fps"]))
    print("  state dim: %d (%d joints%s)" % (
        len(ep["features"]["observation.state"]["names"]),
        len(m["joint_names"]), " + EE" if not args.no_ee else ""))
    print("  hand coverage: left %.0f%%, right %.0f%%   commanding %.0f%%" % (
        cov["left_pct"], cov["right_pct"], m["commanding_pct"]))
    print("  hand QA: %s (score %s)" % (qa.get("grade", "n/a"), qa.get("score", "n/a")))
    print("  wrote %s + %s" % (os.path.basename(rec_path), os.path.basename(meta_path)))


if __name__ == "__main__":
    main()
