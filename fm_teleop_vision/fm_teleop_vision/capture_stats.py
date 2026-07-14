"""capture_stats — pure aggregation of a mirror_datalogger CSV session.

ROS-free so it unit-tests without a graph (mirrors mapping.py). Consumed by capture_browser
to build /capture/index summaries and /capture/detail (dashboard stats + downsampled series
for the GUI charts). Column names match ALL_FIELDS in mirror_datalogger.py.
"""

from __future__ import annotations

import csv
import math
from collections import Counter

# Workspace clamp box (command-frame metres) — the vision.yaml default; overridable so the
# clamp-% reflects the box that was actually in effect for the session.
DEFAULT_WS_MIN = [-0.10, -0.45, -0.55]
DEFAULT_WS_MAX = [0.55, 0.30, 0.25]

# MoveIt Servo StatusCode -> label (matches the debug analysis).
SERVO_NAMES = {
    0: "ok", 1: "decel_sing", 2: "halt_sing", 3: "decel_leaving",
    4: "decel_coll", 5: "halt_coll", 6: "joint_bound",
}

# Columns the GUI charts plot (downsampled in `series`).
CHART_COLS = [
    "err_dist", "err_x", "err_y", "err_z",
    "j1", "j2", "j3", "j4", "j5", "j6", "j7",
    "tgt_x", "tgt_y", "tgt_z", "ee_x", "ee_y", "ee_z",
]

# State columns carried alongside the chart series so the GUI cursor can explain WHY the
# trace looks the way it does at a given instant (engage/tracking/servo/grip).
STATE_COLS = ["engage", "tracking", "commanding_est", "servo_status", "grip"]


def read_csv(path):
    """Load a mirror_log.csv into a list of dict rows (raw strings)."""
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _f(row, name):
    try:
        return float(row.get(name, ""))
    except (ValueError, TypeError):
        return float("nan")


def _col(rows, name):
    return [_f(r, name) for r in rows]


def _stats(xs):
    xs = sorted(x for x in xs if not math.isnan(x))
    if not xs:
        return {"n": 0, "min": 0.0, "max": 0.0, "mean": 0.0, "p50": 0.0, "p95": 0.0}
    n = len(xs)
    return {
        "n": n, "min": xs[0], "max": xs[-1], "mean": sum(xs) / n,
        "p50": xs[min(n - 1, n // 2)], "p95": xs[min(n - 1, int(0.95 * n))],
    }


def _frac_true(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return (sum(1 for x in xs if x >= 0.5) / len(xs)) if xs else 0.0


def summarize(rows, meta=None, ws_min=None, ws_max=None):
    """Full dashboard stats for one session. Quality metrics use commanding rows only."""
    meta = meta or {}
    ws_min = ws_min or DEFAULT_WS_MIN
    ws_max = ws_max or DEFAULT_WS_MAX
    out = {
        "rows": len(rows),
        "duration_s": meta.get("duration_s"),
        "rate_hz": meta.get("rate_hz"),
        "instrumented": meta.get("instrumented"),
        "recorded_bag": meta.get("recorded_bag"),
        "msg_rates_hz": meta.get("msg_rates_hz", {}),
        # Hand-tracking QA (the second data stream's quality layer), computed live by
        # mirror_datalogger and carried through meta.json — None for older sessions.
        "hand_qa": meta.get("hand_qa"),
    }
    if not rows:
        return out

    out["engage_pct"] = round(_frac_true(_col(rows, "engage")) * 100, 1)
    out["tracking_pct"] = round(_frac_true(_col(rows, "tracking")) * 100, 1)
    out["commanding_pct"] = round(_frac_true(_col(rows, "commanding_est")) * 100, 1)

    ci = [r for r in rows if _f(r, "commanding_est") >= 0.5]

    ee = _stats([abs(x) for x in _col(ci, "err_dist")])
    out["err_dist"] = {k: round(v, 4) for k, v in ee.items()}
    for a in ("err_x", "err_y", "err_z"):
        out[a + "_med"] = round(_stats([abs(x) for x in _col(ci, a)])["p50"], 4)

    ss = [int(x) for x in _col(ci, "servo_status") if not math.isnan(x)]
    tot = max(1, len(ss))
    out["servo"] = {SERVO_NAMES.get(k, str(k)): round(v * 100 / tot, 1)
                    for k, v in Counter(ss).most_common()}
    out["servo_ok_pct"] = out["servo"].get("ok", 0.0)

    g = [x for x in _col(ci, "grip") if not math.isnan(x)]
    if g:
        opn = sum(1 for x in g if x < 0.35)
        cls = sum(1 for x in g if x > 0.65)
        out["grip"] = {
            "min": round(min(g), 3), "max": round(max(g), 3),
            "mean": round(sum(g) / len(g), 3),
            "open_pct": round(opn * 100 / len(g), 1),
            "close_pct": round(cls * 100 / len(g), 1),
            "amb_pct": round((len(g) - opn - cls) * 100 / len(g), 1),
        }

    clamp = {}
    for ax, col in enumerate(("tgt_x", "tgt_y", "tgt_z")):
        xs = [x for x in _col(ci, col) if not math.isnan(x)]
        if not xs:
            continue
        lo = sum(1 for x in xs if abs(x - ws_min[ax]) < 0.003)
        hi = sum(1 for x in xs if abs(x - ws_max[ax]) < 0.003)
        clamp["xyz"[ax] + "_min"] = round(lo * 100 / len(xs), 1)
        clamp["xyz"[ax] + "_max"] = round(hi * 100 / len(xs), 1)
    out["clamp"] = clamp

    ranges = {}
    for col in ("hand_dx", "hand_dy", "hand_dz", "moved_x", "moved_y", "moved_z",
                "ee_x", "ee_y", "ee_z"):
        s = _stats(_col(ci, col))
        ranges[col] = round(s["max"] - s["min"], 3)
    out["ranges"] = ranges
    return out


def series(rows, cols=None, n=400):
    """Stride-downsample `cols` (default CHART_COLS + STATE_COLS) to ~n pts, with a t (s) axis."""
    cols = cols or (CHART_COLS + STATE_COLS)
    total = len(rows)
    if total == 0:
        return {"t": []}
    step = max(1, total // n)
    idx = list(range(0, total, step))
    t0 = _f(rows[0], "t_wall")
    out = {"t": [round(_f(rows[i], "t_wall") - t0, 2) for i in idx]}
    for c in cols:
        vals = []
        for i in idx:
            v = _f(rows[i], c)
            vals.append(None if math.isnan(v) else round(v, 4))
        out[c] = vals
    return out


def light_summary(rows, meta=None):
    """Cheap subset for the /capture/index list cards."""
    s = summarize(rows, meta)
    qa = s.get("hand_qa") or {}
    return {
        "rows": s.get("rows"),
        "duration_s": s.get("duration_s"),
        "rate_hz": s.get("rate_hz"),
        "instrumented": s.get("instrumented"),
        "recorded_bag": s.get("recorded_bag"),
        "engage_pct": s.get("engage_pct"),
        "err_p50": s.get("err_dist", {}).get("p50"),
        "err_p95": s.get("err_dist", {}).get("p95"),
        "servo_ok_pct": s.get("servo_ok_pct"),
        "hand_qa_score": qa.get("score"),
        "hand_qa_grade": qa.get("grade"),
        "both_hands_pct": qa.get("both_hands_pct"),
    }
