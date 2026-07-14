"""episode — the data engine: turn a captured mirror session into a training-ready,
self-describing episode that pairs HUMAN hand motion with the ROBOT trajectory it drove.

Pure and ROS-free (like capture_stats.py) so it unit-tests without a graph or a bag. It reads
the session's two aligned logs — `mirror_log.jsonl` (robot: joint positions, actual EE from
tf, the commanded target, engage/tracking) and `hands.jsonl` (both hands' full skeleton) —
joins them on the shared per-tick clock `t_ros` (both are written in the SAME datalogger tick,
so their `t_ros` is identical), and emits one aligned record per timestep. That human-hand
<-> robot-execution pairing on one clock is the part a buyer cannot reconstruct from raw
video, and is why the dataset has an edge.

Per-record schema (LeRobot-flavoured keys):

    t                       seconds from episode start
    observation.state       [<joint positions...>, ee_x, ee_y, ee_z, ee_qw, ee_qx, ee_qy, ee_qz]
    observation.hand.left   {world:[[x,y,z]x21], angles:[...], grip, confidence} | null
    observation.hand.right  same | null
    action                  [tgt_x, tgt_y, tgt_z, tgt_qw, tgt_qx, tgt_qy, tgt_qz, grip]
    engage / tracking / commanding    per-step gating flags

Episode metadata carries the feature schema, robot/frames, fps, frame counts, per-hand
coverage, and the `hand_qa` grade from the recorder. Joint names are read from the log so the
same code handles a single arm or a bimanual capture (both arms' joints) unchanged.

A LeRobotDataset adapter (frames from the bag's /head_cam/image_raw + this table) is a
follow-up; the numeric human<->robot alignment here is the high-value core.
"""

from __future__ import annotations

import json
import os

# Actual EE pose (from tf) and commanded target pose columns in mirror_log.jsonl.
_EE_COLS = ("ee_x", "ee_y", "ee_z", "ee_qw", "ee_qx", "ee_qy", "ee_qz")
_TGT_COLS = ("tgt_x", "tgt_y", "tgt_z", "tgt_qw", "tgt_qx", "tgt_qy", "tgt_qz")


def _read_jsonl(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def load_session(session_dir):
    """Load a capture session dir into {meta, log, hands, dir}. Missing files -> empty."""
    def _load(name, reader, default):
        p = os.path.join(session_dir, name)
        return reader(p) if os.path.exists(p) else default
    meta = _load("meta.json", lambda p: json.load(open(p)), {})
    log = _load("mirror_log.jsonl", _read_jsonl, [])
    hands = _load("hands.jsonl", _read_jsonl, [])
    return {"meta": meta, "log": log, "hands": hands, "dir": session_dir}


def _num(v):
    """Parse a log cell to float; "" / None / non-numeric -> None (missing)."""
    if v is None or v == "":
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _hand_obs(h):
    """Reduce a hands.jsonl hand block to the training observation: 3D world skeleton +
    per-finger joint angles + grip + confidence (image pixels dropped — they belong with the
    video, not the proprio vector)."""
    if not h:
        return None
    return {
        "world": h.get("world"),            # [[x, y, z] x 21], metres, wrist origin
        "angles": h.get("joint_angles"),    # [19] per-finger flexion + abduction (rad)
        "grip": h.get("grip"),
        "confidence": h.get("confidence"),
    }


def _tkey(row):
    return round(float(row.get("t_ros") or row.get("t_wall") or 0.0), 4)


def build_episode(session, *, include_ee=True):
    """Join the robot log with both hands into aligned per-timestep records + a feature schema
    + episode metadata. The robot log is the master timeline (one row per tick); each hand is
    attached when its skeleton was fresh that tick (else null)."""
    log = session.get("log") or []
    hands_by_t = {_tkey(h): h for h in (session.get("hands") or [])}

    joint_names = sorted((log[0].get("joints") or {}).keys()) if log else []
    t0 = float(log[0].get("t_ros") or log[0].get("t_wall") or 0.0) if log else 0.0

    records = []
    for row in log:
        joints = row.get("joints") or {}
        state = [_num(joints.get(n)) for n in joint_names]
        if include_ee:
            state += [_num(row.get(c)) for c in _EE_COLS]
        action = [_num(row.get(c)) for c in _TGT_COLS] + [_num(row.get("grip"))]
        h = hands_by_t.get(_tkey(row)) or {}
        records.append({
            "t": round(float(row.get("t_ros") or 0.0) - t0, 4),
            "observation.state": state,
            "observation.hand.left": _hand_obs(h.get("left")),
            "observation.hand.right": _hand_obs(h.get("right")),
            "action": action,
            "engage": int(row.get("engage") or 0),
            "tracking": int(row.get("tracking") or 0),
            "commanding": int(row.get("commanding_est") or 0),
        })
    return {
        "records": records,
        "features": _features(joint_names, include_ee),
        "meta": _episode_meta(session, records, joint_names),
    }


def _features(joint_names, include_ee):
    ee_names = list(_EE_COLS) if include_ee else []
    hand_desc = {"dtype": "json", "shape": [],
                 "names": ["world[21x3]", "angles[19]", "grip", "confidence"]}
    return {
        "observation.state": {
            "dtype": "float32", "shape": [len(joint_names) + len(ee_names)],
            "names": list(joint_names) + ee_names,
        },
        "observation.hand.left": hand_desc,
        "observation.hand.right": dict(hand_desc),
        "action": {
            "dtype": "float32", "shape": [len(_TGT_COLS) + 1],
            "names": list(_TGT_COLS) + ["grip"],
        },
    }


def _episode_meta(session, records, joint_names):
    meta = session.get("meta") or {}
    n = len(records)
    dur = records[-1]["t"] if records else 0.0
    left = sum(1 for r in records if r["observation.hand.left"])
    right = sum(1 for r in records if r["observation.hand.right"])
    commanding = sum(r["commanding"] for r in records)
    return {
        "session": meta.get("session"),
        "robot_frames": {"command_frame": meta.get("command_frame"),
                         "ee_frame": meta.get("ee_frame")},
        "joint_names": joint_names,
        "frames": n,
        "duration_s": round(dur, 2),
        "fps": round(n / dur, 1) if dur > 0 else meta.get("rate_hz"),
        "hand_coverage": {
            "left_pct": round(100.0 * left / n, 1) if n else 0.0,
            "right_pct": round(100.0 * right / n, 1) if n else 0.0,
        },
        "commanding_pct": round(100.0 * commanding / n, 1) if n else 0.0,
        "hand_qa": meta.get("hand_qa"),
    }


def write_jsonl(episode, path):
    """Write one aligned record per line — universal, dependency-free."""
    with open(path, "w") as f:
        for r in episode["records"]:
            f.write(json.dumps(r) + "\n")


def write_meta(episode, path):
    """Write the episode descriptor (feature schema + metadata) next to the records."""
    with open(path, "w") as f:
        json.dump({"features": episode["features"], **episode["meta"]}, f, indent=2)


def write_parquet(episode, path):
    """Optional columnar export (needs pyarrow). State/action become float columns; the two
    hand skeletons are stored as JSON strings so the row stays flat and portable."""
    import pyarrow as pa  # optional dependency
    import pyarrow.parquet as pq

    recs = episode["records"]
    cols = {
        "t": [r["t"] for r in recs],
        "observation.state": [r["observation.state"] for r in recs],
        "action": [r["action"] for r in recs],
        "observation.hand.left": [json.dumps(r["observation.hand.left"]) for r in recs],
        "observation.hand.right": [json.dumps(r["observation.hand.right"]) for r in recs],
        "engage": [r["engage"] for r in recs],
        "tracking": [r["tracking"] for r in recs],
        "commanding": [r["commanding"] for r in recs],
    }
    pq.write_table(pa.table(cols), path)
