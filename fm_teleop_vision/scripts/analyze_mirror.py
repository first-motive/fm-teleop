#!/usr/bin/env python3
"""analyze_mirror — turn a mirror_datalogger capture into a debugging summary + plots.

Reads mirror_log.csv from a capture session and answers the core question: *how did the
hand motion become the robot motion?* It prints per-axis ranges, the empirically-measured
hand->target axis mapping (correlation), workspace-clamp saturation, and the servo
tracking error (commanded target vs actually-reached EE), then writes PNG plots.

    python3 analyze_mirror.py /ws/data/teleop_captures/<session>

stdlib + numpy + matplotlib (Agg) — no pandas. Plots land in <session>/plots/.
Robust to missing dbg_* columns (uncomment = not instrumented; internals are estimates).
"""

from __future__ import annotations

import csv
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _col(rows, name):
    """Column as a float array with '' -> nan."""
    out = []
    for r in rows:
        v = r.get(name, "")
        try:
            out.append(float(v))
        except (ValueError, TypeError):
            out.append(float("nan"))
    return np.array(out, dtype=float)


def load(session):
    path = os.path.join(session, "mirror_log.csv")
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("no rows in %s" % path)
    return rows, rows[0].keys()


def _rng(a):
    a = a[~np.isnan(a)]
    if a.size == 0:
        return "n/a"
    return "min %+.4f  max %+.4f  span %.4f" % (a.min(), a.max(), a.max() - a.min())


def _corr(a, b):
    m = ~(np.isnan(a) | np.isnan(b))
    if m.sum() < 3 or np.std(a[m]) < 1e-9 or np.std(b[m]) < 1e-9:
        return float("nan")
    return float(np.corrcoef(a[m], b[m])[0, 1])


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: analyze_mirror.py <session-dir>")
    session = sys.argv[1]
    rows, cols = load(session)
    # dbg_* columns always exist in the fixed schema; "instrumented" means they were
    # actually populated (mirror_source published /vision/mirror_debug during commanding).
    dbg_present = any(c.startswith("dbg_") for c in cols)
    has_dbg = dbg_present and bool(np.nansum(np.abs(_col(rows, "dbg_commanding"))) > 0)

    t = _col(rows, "t_ros")
    t = t - np.nanmin(t)
    commanding = _col(rows, "commanding_est")
    cmd = commanding > 0.5
    n_cmd = int(cmd.sum())

    print("=" * 72)
    print("mirror capture: %s" % session)
    print("rows=%d  commanding=%d (%.1f%%)  duration=%.1fs  instrumented=%s"
          % (len(rows), n_cmd, 100.0 * n_cmd / len(rows), t.max(), has_dbg))
    print("=" * 72)
    if n_cmd < 5:
        print("Almost no commanding rows — was a hand tracked AND engaged during capture?")
        print("(engage=true + hand in frame is required for target_pose / mirror_debug.)")

    # --- restrict to the commanding segment (where the mapping is live) ---
    def c(name):
        a = _col(rows, name)
        a[~cmd] = np.nan
        return a

    hand_d = {ax: c("hand_d" + ax) for ax in "xyz"}          # hand offset, image-width units
    moved = {ax: c("moved_" + ax) for ax in "xyz"}           # commanded EE offset, metres
    ee_off = {ax: c("ee_" + ax) - c("ee_ref_" + ax) for ax in "xyz"}  # actual EE offset, metres
    err_dist = c("err_dist")

    print("\nHAND input offset since engage (normalized image-width units):")
    for ax in "xyz":
        print("  hand_d%s : %s" % (ax, _rng(hand_d[ax])))
    print("\nCOMMANDED EE offset (target - ee_ref, metres):")
    for ax in "xyz":
        print("  moved_%s : %s" % (ax, _rng(moved[ax])))
    print("\nACTUAL EE offset reached (tf - ee_ref, metres):")
    for ax in "xyz":
        print("  ee_off_%s: %s" % (ax, _rng(ee_off[ax])))
    print("\nSERVO tracking error |target - actual EE| (metres):")
    print("  err_dist: %s  mean %.4f" % (_rng(err_dist), np.nanmean(err_dist)))

    # --- empirical hand->target axis mapping (correlation matrix) ---
    print("\nHAND->TARGET axis mapping (corr of hand_d[i] vs moved[j]); ~+/-1 = that axis pair):")
    print("            moved_x   moved_y   moved_z")
    for hi in "xyz":
        cells = "  ".join("%+7.2f" % _corr(hand_d[hi], moved[hj]) for hj in "xyz")
        print("  hand_d%s   %s" % (hi, cells))
    print("  (config axis_map_linear=[z,x,-y], axis_gain=[1,1,0] -> depth/hand_dz should"
          " map to nothing; a flat moved_z confirms depth is OFF.)")

    if has_dbg:
        ofl = {ax: c("dbg_overflow_" + ax) for ax in "xyz"}
        wm = c("dbg_w_m")
        print("\nWORKSPACE clamp overflow (metres past the box; nonzero = motion eaten):")
        for ax in "xyz":
            a = ofl[ax][~np.isnan(ofl[ax])]
            frac = float((np.abs(a) > 1e-6).mean()) if a.size else 0.0
            print("  overflow_%s: %s  (clamped %.0f%% of ticks)" % (ax, _rng(ofl[ax]), 100 * frac))
        wmv = wm[~np.isnan(wm)]
        if wmv.size:
            print("  W_m (m per image-width): mean %.3f  (-1 = degenerate/fallback)"
                  % np.nanmean(wmv[wmv > 0]) if (wmv > 0).any() else "  W_m: all fallback")

    # --- plots ---
    plot_dir = os.path.join(session, "plots")
    os.makedirs(plot_dir, exist_ok=True)

    fig, axs = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
    for ax in "xyz":
        axs[0].plot(t, hand_d[ax], label="hand_d" + ax)
    axs[0].set_title("HAND input offset since engage (image-width units)")
    axs[0].legend(loc="upper right"); axs[0].grid(alpha=0.3)
    for ax in "xyz":
        axs[1].plot(t, moved[ax], label="moved_" + ax)
    axs[1].set_title("COMMANDED EE offset (target - ee_ref, m)")
    axs[1].legend(loc="upper right"); axs[1].grid(alpha=0.3)
    for ax in "xyz":
        axs[2].plot(t, moved[ax], "--", alpha=0.6, label="cmd_" + ax)
        axs[2].plot(t, ee_off[ax], label="actual_" + ax)
    axs[2].set_title("COMMANDED (--) vs ACTUAL EE offset (m) — gap = servo lag/limit")
    axs[2].legend(loc="upper right", ncol=2); axs[2].grid(alpha=0.3)
    axs[3].plot(t, err_dist, color="crimson", label="|target-actual| (m)")
    if has_dbg:
        for ax in "xyz":
            axs[3].plot(t, c("dbg_overflow_" + ax), alpha=0.6, label="overflow_" + ax)
    axs[3].set_title("Servo tracking error + workspace overflow (m)")
    axs[3].set_xlabel("t (s)"); axs[3].legend(loc="upper right"); axs[3].grid(alpha=0.3)
    fig.tight_layout()
    p1 = os.path.join(plot_dir, "timeseries.png")
    fig.savefig(p1, dpi=110); plt.close(fig)

    # hand vs target scatter per dominant mapped axis pair
    fig2, axs2 = plt.subplots(1, 3, figsize=(15, 5))
    pairs = [("x", "moved's best"), ("y", ""), ("z", "")]
    for i, hi in enumerate("xyz"):
        # find the target axis most correlated with this hand axis
        corrs = {hj: _corr(hand_d[hi], moved[hj]) for hj in "xyz"}
        best = max(corrs, key=lambda k: abs(corrs[k]) if not np.isnan(corrs[k]) else -1)
        axs2[i].scatter(hand_d[hi], moved[best], s=6, alpha=0.4)
        axs2[i].set_xlabel("hand_d%s (img units)" % hi)
        axs2[i].set_ylabel("moved_%s (m)" % best)
        axs2[i].set_title("hand_d%s -> moved_%s  (r=%.2f)" % (hi, best, corrs[best]))
        axs2[i].grid(alpha=0.3)
    fig2.tight_layout()
    p2 = os.path.join(plot_dir, "hand_vs_target_scatter.png")
    fig2.savefig(p2, dpi=110); plt.close(fig2)

    print("\nplots written:\n  %s\n  %s" % (p1, p2))


if __name__ == "__main__":
    main()
