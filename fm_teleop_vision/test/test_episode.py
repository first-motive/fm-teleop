"""Pure unit tests for fm_teleop_vision.episode — the data engine, no ROS graph or bag.

Builds a tiny synthetic session (mirror_log.jsonl + hands.jsonl + meta.json) on disk and
checks the human-hand <-> robot join on the shared tick clock t_ros.
"""

import json
import os

from fm_teleop_vision import episode


def _write_session(tmp_path):
    d = str(tmp_path)
    # two ticks; t_ros is the shared join key (same value in both files per tick).
    log = [
        {"t_wall": 100.01, "t_ros": 10.0, "engage": 1, "tracking": 1, "commanding_est": 1,
         "tgt_x": 0.2, "tgt_y": -0.1, "tgt_z": -0.2, "tgt_qw": 1.0, "tgt_qx": 0.0,
         "tgt_qy": 0.0, "tgt_qz": 0.0, "ee_x": 0.19, "ee_y": -0.1, "ee_z": -0.2,
         "ee_qw": 1.0, "ee_qx": 0.0, "ee_qy": 0.0, "ee_qz": 0.0, "grip": 0.6,
         "joints": {"openarm_right_joint1": 0.1, "openarm_right_joint2": 0.5}},
        {"t_wall": 100.03, "t_ros": 10.02, "engage": 1, "tracking": 1, "commanding_est": 0,
         "tgt_x": 0.21, "tgt_y": -0.1, "tgt_z": -0.2, "tgt_qw": 1.0, "tgt_qx": 0.0,
         "tgt_qy": 0.0, "tgt_qz": 0.0, "ee_x": 0.20, "ee_y": -0.1, "ee_z": -0.2,
         "ee_qw": 1.0, "ee_qx": 0.0, "ee_qy": 0.0, "ee_qz": 0.0, "grip": 0.7,
         "joints": {"openarm_right_joint1": 0.11, "openarm_right_joint2": 0.51}},
    ]
    hands = [  # only the FIRST tick has a fresh right hand; the second has none
        {"t_wall": 100.011, "t_ros": 10.0,
         "right": {"hand": "right", "confidence": 0.95,
                   "world": [[0.0, 0.0, 0.0]] * 21, "image": [[1.0, 2.0]] * 21,
                   "palm_quat": [1.0, 0.0, 0.0, 0.0], "joint_angles": [0.0] * 19,
                   "grip": 0.6, "age": 0.0},
         "left": None},
    ]
    with open(os.path.join(d, "mirror_log.jsonl"), "w") as f:
        for r in log:
            f.write(json.dumps(r) + "\n")
    with open(os.path.join(d, "hands.jsonl"), "w") as f:
        for r in hands:
            f.write(json.dumps(r) + "\n")
    with open(os.path.join(d, "meta.json"), "w") as f:
        json.dump({"session": "s", "rate_hz": 50.0, "command_frame": "openarm_right_base_link",
                   "ee_frame": "openarm_right_link7",
                   "hand_qa": {"score": 0.8, "grade": "B"}}, f)
    return d


def test_build_episode_joins_hand_to_robot_on_t_ros(tmp_path):
    ep = episode.build_episode(episode.load_session(_write_session(tmp_path)))
    recs = ep["records"]
    assert len(recs) == 2
    # tick 0: right hand present (t_ros 10.0 matched), left absent
    assert recs[0]["observation.hand.right"] is not None
    assert recs[0]["observation.hand.left"] is None
    assert len(recs[0]["observation.hand.right"]["world"]) == 21
    assert len(recs[0]["observation.hand.right"]["angles"]) == 19
    # tick 1: no hands.jsonl row for t_ros 10.02 -> both null
    assert recs[1]["observation.hand.right"] is None


def test_state_and_action_vectors(tmp_path):
    ep = episode.build_episode(episode.load_session(_write_session(tmp_path)))
    feats = ep["features"]
    # 2 joints + 7 EE dims
    assert feats["observation.state"]["shape"] == [9]
    assert feats["observation.state"]["names"][:2] == [
        "openarm_right_joint1", "openarm_right_joint2"]
    assert feats["action"]["shape"] == [8]                 # 7 target pose + grip
    r0 = ep["records"][0]
    assert r0["observation.state"] == [0.1, 0.5, 0.19, -0.1, -0.2, 1.0, 0.0, 0.0, 0.0]
    assert r0["action"] == [0.2, -0.1, -0.2, 1.0, 0.0, 0.0, 0.0, 0.6]
    assert r0["t"] == 0.0 and ep["records"][1]["t"] == 0.02


def test_episode_meta_coverage_and_qa(tmp_path):
    ep = episode.build_episode(episode.load_session(_write_session(tmp_path)))
    m = ep["meta"]
    assert m["frames"] == 2
    assert m["hand_coverage"]["right_pct"] == 50.0        # 1 of 2 ticks
    assert m["hand_coverage"]["left_pct"] == 0.0
    assert m["commanding_pct"] == 50.0                    # 1 of 2 ticks commanding
    assert m["hand_qa"]["grade"] == "B"
    assert m["joint_names"] == ["openarm_right_joint1", "openarm_right_joint2"]


def test_no_ee_drops_pose(tmp_path):
    ep = episode.build_episode(episode.load_session(_write_session(tmp_path)), include_ee=False)
    assert ep["features"]["observation.state"]["shape"] == [2]   # joints only
    assert ep["records"][0]["observation.state"] == [0.1, 0.5]


def test_write_jsonl_and_meta_roundtrip(tmp_path):
    ep = episode.build_episode(episode.load_session(_write_session(tmp_path)))
    rec_p = os.path.join(str(tmp_path), "episode.jsonl")
    meta_p = os.path.join(str(tmp_path), "episode.json")
    episode.write_jsonl(ep, rec_p)
    episode.write_meta(ep, meta_p)
    with open(rec_p) as f:
        lines = [json.loads(x) for x in f if x.strip()]
    assert len(lines) == 2 and lines[0]["observation.hand.right"]["confidence"] == 0.95
    with open(meta_p) as f:
        meta = json.load(f)
    assert "features" in meta and meta["frames"] == 2
