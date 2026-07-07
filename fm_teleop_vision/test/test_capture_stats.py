"""Pure unit tests for fm_teleop_vision.capture_stats — no ROS graph, no CSV file needed."""

from fm_teleop_vision import capture_stats


def _row(**kw):
    # every ALL_FIELDS column the stats read; unspecified -> "" (parsed as NaN).
    base = {c: "" for c in (
        "t_wall", "engage", "tracking", "commanding_est", "err_dist",
        "err_x", "err_y", "err_z", "servo_status", "grip",
        "tgt_x", "tgt_y", "tgt_z", "ee_x", "ee_y", "ee_z",
        "hand_dx", "hand_dy", "hand_dz", "moved_x", "moved_y", "moved_z",
        "j1", "j2", "j3", "j4", "j5", "j6", "j7",
    )}
    base.update({k: str(v) for k, v in kw.items()})
    return base


def _session():
    rows = []
    for i in range(100):
        commanding = i >= 20  # first 20 idle, rest commanding
        rows.append(_row(
            t_wall=1000.0 + i * 0.02,
            engage=1 if commanding else 0,
            tracking=1 if commanding else 0,
            commanding_est=1 if commanding else 0,
            err_dist=0.05 if commanding else 0.0,
            servo_status=0 if i % 2 else 1,   # half ok, half decel_sing over commanding
            grip=0.8 if commanding else 0.2,
            tgt_y=-0.45 if commanding else -0.26,   # commanding rows pinned at y-min face
        ))
    return rows


def test_summarize_percentages_and_error():
    rows = _session()
    s = capture_stats.summarize(rows, meta={"duration_s": 2.0, "rate_hz": 50.0})
    assert s["rows"] == 100
    assert s["engage_pct"] == 80.0
    assert s["commanding_pct"] == 80.0
    assert s["err_dist"]["p50"] == 0.05        # commanding rows only
    assert s["err_dist"]["n"] == 80


def test_summarize_servo_and_grip_and_clamp():
    s = capture_stats.summarize(_session())
    # servo: commanding rows alternate 0/1 -> ~50/50
    assert 40 <= s["servo"]["ok"] <= 60
    assert s["servo_ok_pct"] == s["servo"]["ok"]
    # grip 0.8 on all commanding rows -> all "close"
    assert s["grip"]["close_pct"] == 100.0
    assert s["grip"]["open_pct"] == 0.0
    # tgt_y pinned at ws_min[1]=-0.45 on every commanding row -> 100% y_min clamp
    assert s["clamp"]["y_min"] == 100.0


def test_summarize_empty():
    s = capture_stats.summarize([])
    assert s["rows"] == 0
    assert "err_dist" not in s


def test_series_downsamples_with_time_axis():
    rows = _session()
    ser = capture_stats.series(rows, cols=["err_dist"], n=25)
    assert len(ser["t"]) == len(ser["err_dist"]) <= 26
    assert ser["t"][0] == 0.0                 # relative to first t_wall
    assert ser["t"][-1] > 0.0


def test_light_summary_subset():
    ls = capture_stats.light_summary(_session(), meta={"duration_s": 2.0})
    assert ls["rows"] == 100 and ls["engage_pct"] == 80.0
    assert ls["err_p95"] == 0.05 and "servo_ok_pct" in ls
