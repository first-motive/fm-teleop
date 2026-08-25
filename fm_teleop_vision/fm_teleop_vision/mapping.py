"""Vision retarget math — pure, ROS-free, unit-tested without a camera or a graph.

The control half of the vision mirror: it takes the hand values the perception side
produces (see ``fm_data_perception.landmarks``) and turns them into a robot command.

    metric_mirror_target(...) -> indexed ~1:1 metric pose mirroring (+ clamp overflow)
    image_width_m / metric_scale / depth_delta_m -> pinhole image-units -> metres
    box_clamp / step_limit    -> keep the target in the dexterous region, bound its rate
    grip_preset(...)          -> curl + hysteresis -> "open" | "close"

These implement an ABSOLUTE target latched to a REFERENCE pose (captured at clutch
engage), not raw frame-to-frame deltas: a steady hand maps to a steady target, so a
steady bias becomes a bounded offset rather than integrated drift.
"""

from __future__ import annotations

import math

from fm_teleop_core import retarget


def _sub(a, b):
    """Component-wise a - b for two 3-tuples."""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _clamp01(v):
    return 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)


# --- axis remap (operator/vision frame -> robot command frame) -----------------------


def parse_axis_map(spec):
    """Parse e.g. ["z","x","-y"] into [(2,+1),(0,+1),(1,-1)] (source index, sign)."""
    out = []
    for token in spec:
        token = str(token).strip()
        sign = -1.0 if token.startswith("-") else 1.0
        axis = token[-1].lower()
        if axis not in ("x", "y", "z"):
            raise ValueError(f"Bad axis token {token!r}; use x|y|z with optional '-'.")
        out.append(({"x": 0, "y": 1, "z": 2}[axis], sign))
    if len(out) != 3:
        raise ValueError(f"axis_map must have 3 entries, got {len(out)}.")
    return out


def remap(vec, parsed_axis_map):
    """Reorder/sign a 3-vector by a parsed axis map."""
    return tuple(parsed_axis_map[i][1] * vec[parsed_axis_map[i][0]] for i in range(3))



def _seq3(v):
    """Accept a scalar (broadcast) or a length-3 sequence; return a 3-tuple of floats."""
    if isinstance(v, (int, float)):
        return (float(v), float(v), float(v))
    out = tuple(float(x) for x in v)
    if len(out) != 3:
        raise ValueError(f"expected a scalar or 3 values, got {len(out)}.")
    return out


# --- control: absolute pose mirroring (for MoveIt Servo PoseTracking) ----------------


def mirror_target(ee_ref_pos, hand_ref, hand_now, axis_map, *, scale, workspace_box):
    """Absolute EE target POSITION for indexed 1:1 hand mirroring (mouse-pickup model).

        target = clamp_box( ee_ref_pos + remap( scale * (hand_now - hand_ref) ) )

    Unlike ``linear_velocity`` (which emits a velocity from the clutch offset, so holding the
    hand off-centre keeps the arm gliding), this emits an ABSOLUTE target the EE servos to and
    HOLDS: a steady hand maps to a steady target, so a steady bias in the hand signal becomes a
    bounded offset rather than integrated drift.

      ``ee_ref_pos``    (x,y,z) EE position in the command/planning frame, latched at engage.
      ``hand_ref``      (x,y,z) hand position (normalized image units) latched at engage.
      ``hand_now``      (x,y,z) current hand position (normalized image units).
      ``axis_map``      parsed axis map (image axes -> command-frame axes), as parse_axis_map.
      ``scale``         metres per normalized-image unit; scalar or length-3 (per input axis).
                        The depth axis is the weakest (mono proxy) — default its scale to 0 for
                        camera-plane-only mirroring.
      ``workspace_box`` ((xmin,ymin,zmin), (xmax,ymax,zmax)) per-axis clamp in the command frame;
                        keeps the target inside the dexterous region (away from the straight-arm
                        singularity / joint limits).

    Returns the (x,y,z) command-frame target. Orientation is held = ee_ref_quat by the caller
    (angular off), so only position is produced here. Scale is applied in the INPUT axes BEFORE
    the remap (like ``_shape3`` applies gain before remap), so signs stay tunable via axis_map.
    """
    delta = _sub(hand_now, hand_ref)
    s = _seq3(scale)
    moved = remap((delta[0] * s[0], delta[1] * s[1], delta[2] * s[2]), axis_map)
    lo, hi = workspace_box
    return tuple(retarget.clamp(ee_ref_pos[i] + moved[i], lo[i], hi[i]) for i in range(3))


# --- control: metric mirroring (image units -> metres via the apparent hand size) ----

_MIN_HAND_Z = 1e-3  # below this the apparent-size proxy is degenerate (lost/collapsed hand)
_NO_BOX = ((-math.inf, -math.inf, -math.inf), (math.inf, math.inf, math.inf))


def image_width_m(hand_z, *, hand_span_m, min_z=_MIN_HAND_Z):
    """Image width in METRES at the hand's distance, or None if ``hand_z`` is degenerate.

    Pinhole geometry: span_px / width_px == span_m / width_m, and the published depth
    proxy is exactly hand_z = span_px / width_px, so width_m = hand_span_m / hand_z.
    Latched at engage, this converts normalized-image-width units into metres of REAL
    hand travel — the mirror amplitude no longer depends on how far the operator stands
    from the camera.
    """
    if hand_z is None or hand_z < min_z:
        return None
    return hand_span_m / hand_z


def metric_scale(width_m, *, mirror_gain, axis_gain, fallback_scale):
    """Per-input-axis scale triple for ``metric_mirror_target``.

    Returns (sx, sy, depth_gain): sx/sy in metres per normalized-image-width unit
    (mirror_gain * width_m * axis_gain), so mirror_gain=1, axis_gain=1 is ~1:1 physical
    mirroring; ``fallback_scale`` (m/unit) stands in for width_m when it is None
    (degenerate hand size at engage). depth_gain is UNITLESS (mirror_gain * axis_gain[2])
    because it multiplies ``depth_delta_m``, which is already metres.
    """
    g = _seq3(axis_gain)
    w = fallback_scale if width_m is None else width_m
    return (mirror_gain * w * g[0], mirror_gain * w * g[1], mirror_gain * g[2])


def depth_delta_m(hand_ref_z, hand_now_z, *, hand_span_m, min_z=_MIN_HAND_Z):
    """Metric depth motion since engage; POSITIVE = the hand moved TOWARD the camera.

    distance = hand_span_m / hand_z (the same pinhole relation as ``image_width_m``), so
    delta = hand_span_m * (1/z_ref - 1/z_now). Toward the camera the apparent size grows,
    matching the sign of the raw proxy delta — existing axis_map sign tuning carries over.
    Returns 0.0 if either size is degenerate (never inject a spike from a lost hand).
    """
    if hand_ref_z is None or hand_now_z is None:
        return 0.0
    if hand_ref_z < min_z or hand_now_z < min_z:
        return 0.0
    return hand_span_m * (1.0 / hand_ref_z - 1.0 / hand_now_z)


def box_clamp(pos, box):
    """Clamp ``pos`` into ``box``; return (clamped, overflow) with overflow = pos - clamped.

    overflow[i] is the signed metres the raw target lies beyond the box face (0.0 when
    inside), so the caller can LOG saturation — a silently clamped workspace box reads as
    "the arm barely moves" to the operator.
    """
    lo, hi = box
    clamped = tuple(retarget.clamp(pos[i], lo[i], hi[i]) for i in range(3))
    overflow = tuple(pos[i] - clamped[i] for i in range(3))
    return clamped, overflow


def metric_mirror_target(ee_ref_pos, hand_ref, hand_now, axis_map, *,
                         scale, hand_span_m, workspace_box):
    """``mirror_target`` in METRIC units: ~1:1 physical mirroring with observable clamping.

    ``scale`` is the ``metric_scale`` triple (m/unit, m/unit, unitless depth gain). The
    raw z delta (apparent size) is replaced by ``depth_delta_m`` (metres toward the
    camera), so all three input axes are metres of real hand travel before the axis_map
    remap. Reuses ``mirror_target`` unchanged with an infinite box and applies the
    workspace clamp separately via ``box_clamp``.

    Returns (target, overflow) — overflow per ``box_clamp``.
    """
    dz = depth_delta_m(hand_ref[2], hand_now[2], hand_span_m=hand_span_m)
    hand_in = (hand_now[0], hand_now[1], hand_ref[2] + dz)
    raw = mirror_target(ee_ref_pos, hand_ref, hand_in, axis_map,
                        scale=scale, workspace_box=_NO_BOX)
    return box_clamp(raw, workspace_box)


def step_limit(prev, target, max_step):
    """Rate-limit a per-tick target: cap each component of (target - prev) to +/-max_step.

    A single noisy landmark frame otherwise pushes the mirror target ~20 mm in one tick
    (the ~0.8 m/unit engage scale amplifies MediaPipe jitter); clamping the step at a fixed
    metric budget per tick turns that spike into a bounded slew without touching the steady
    signal. ``prev`` is the last published target; ``max_step`` is metres per tick (so the
    EE speed ceiling is max_step * publish_rate). Disabled — returns ``target`` unchanged —
    when ``prev`` is None (first tick after engage) or ``max_step`` <= 0.
    """
    if prev is None or max_step <= 0.0:
        return tuple(target)
    return tuple(prev[i] + retarget.clamp(target[i] - prev[i], -max_step, max_step)
                 for i in range(3))


# --- control: gripper ----------------------------------------------------------------


def grip_preset(curl, prev_state, *, open_below=0.35, close_above=0.65):
    """Hysteresis: 'close' once curl exceeds close_above, 'open' once below open_below.

    Between the thresholds the previous state is held (no chatter). prev_state defaults
    to 'open' when None.
    """
    state = prev_state or "open"
    if curl >= close_above:
        return "close"
    if curl <= open_below:
        return "open"
    return state


def grip_position(curl, *, open_pos, close_pos):
    """Linear map curl in [0,1] -> a gripper joint target between open_pos and close_pos."""
    c = _clamp01(curl)
    return open_pos + (close_pos - open_pos) * c
