"""Vision retarget math — pure, ROS-free, unit-tested without a camera or a graph.

This is the heart of vision teleop, kept free of rclpy/mediapipe so it can be tested in
isolation (mirrors how fm_teleop_device/hand_presets.py and fm_teleop_core/retarget.py
split pure math out of the nodes). Two groups of functions:

PERCEPTION GEOMETRY (called by the tracker node to reduce raw hand landmarks):
    palm_orientation(...)  -> a quaternion for the hand's pose
    finger_curl(...)       -> a 0..1 openness/curl scalar for the gripper
    control_position(...)  -> wrist pixels -> (x, y, depth) in normalized-image-WIDTH units

CONTROL (called by the source node every tick):
    linear_velocity(...)   -> clutch-referenced position error -> unitless [-1,1] twist
    angular_velocity(...)  -> clutch-referenced orientation error -> unitless [-1,1] twist
    metric_mirror_target(...) -> indexed ~1:1 metric pose mirroring (+ clamp overflow)
    image_width_m / metric_scale / depth_delta_m -> pinhole image-units -> metres
    grip_preset(...)       -> curl + hysteresis -> "open" | "close"

The control functions implement a P-controller on the offset from a latched REFERENCE
pose (captured at clutch engage), NOT raw frame-to-frame deltas: rate-independent, self-
centering (hand back at the reference -> zero command), no integrated drift. MoveIt Servo
consumes the result as a unitless velocity ([-1,1], it applies its own scale), so the
source must emit this on a steady timer; zero when disengaged/stale.
"""

from __future__ import annotations

import math

from fm_teleop_core import retarget

# --- small vector helpers (3-tuples; no numpy so the test needs no extra deps) -------


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a):
    return math.sqrt(_dot(a, a))


def _normalize(a):
    n = _norm(a)
    if n < 1e-9:
        return None
    return (a[0] / n, a[1] / n, a[2] / n)


# --- quaternion helpers (w, x, y, z) -------------------------------------------------

IDENTITY_QUAT = (1.0, 0.0, 0.0, 0.0)


def quat_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-9:
        return IDENTITY_QUAT
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_from_axes(x_axis, y_axis, z_axis):
    """Quaternion for the rotation whose columns are the given orthonormal axes."""
    r00, r01, r02 = x_axis[0], y_axis[0], z_axis[0]
    r10, r11, r12 = x_axis[1], y_axis[1], z_axis[1]
    r20, r21, r22 = x_axis[2], y_axis[2], z_axis[2]
    t = r00 + r11 + r22
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s
    return quat_normalize((qw, qx, qy, qz))


def quat_to_rotvec(q):
    """Rotation vector (axis * angle, radians) for quaternion q (shortest-arc)."""
    w, x, y, z = quat_normalize(q)
    if w < 0.0:  # take the shortest path
        w, x, y, z = -w, -x, -y, -z
    v_norm = math.sqrt(x * x + y * y + z * z)
    if v_norm < 1e-9:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(v_norm, w)
    k = angle / v_norm
    return (x * k, y * k, z * k)


# --- perception geometry -------------------------------------------------------------


def palm_orientation(wrist, index_mcp, middle_mcp, pinky_mcp):
    """Quaternion of the hand frame from four palm points (world metres).

    x = wrist -> middle knuckle (along the hand); z = palm normal; y completes a
    right-handed frame. Returns IDENTITY_QUAT if the points are degenerate.
    """
    x_axis = _normalize(_sub(middle_mcp, wrist))
    across = _normalize(_sub(pinky_mcp, index_mcp))
    if x_axis is None or across is None:
        return IDENTITY_QUAT
    z_axis = _normalize(_cross(x_axis, across))
    if z_axis is None:
        return IDENTITY_QUAT
    y_axis = _cross(z_axis, x_axis)
    return quat_from_axes(x_axis, y_axis, z_axis)


def finger_curl(wrist, middle_mcp, mcps, tips, *, open_ref=1.25, closed_ref=0.35):
    """Return curl in [0,1]: 0 = fingers extended, 1 = fully curled.

    openness = mean(|mcp_i - tip_i|) / |wrist - middle_mcp| (a scale-invariant ratio,
    large when extended, small when curled). Mapped through (open_ref, closed_ref) and
    clamped. The two refs are empirical and exposed as params for tuning.
    """
    scale = _norm(_sub(middle_mcp, wrist))
    if scale < 1e-6 or not mcps or len(mcps) != len(tips):
        return 0.0
    openness = sum(_norm(_sub(t, m)) for m, t in zip(mcps, tips)) / len(tips) / scale
    span = open_ref - closed_ref
    if abs(span) < 1e-9:
        return 0.0
    return _clamp01((open_ref - openness) / span)


def _clamp01(v):
    return 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)


def control_position(wrist_px, middle_mcp_px, image_w):
    """(x, y, depth) control position in normalized-image-WIDTH units from pixel landmarks.

    ALL components are normalized by the image WIDTH — x in [0,1], y in [0, h/w] — so one
    metres-per-unit factor (``image_width_m`` at the hand's distance) applies uniformly to
    every axis. (Normalizing y by the height would shrink vertical motion by the aspect
    ratio relative to horizontal.) depth = the apparent hand size (wrist -> middle MCP)
    over the width: the mono depth proxy (hand bigger = closer).
    """
    wx_px, wy_px = wrist_px
    mx_px, my_px = middle_mcp_px
    size = ((mx_px - wx_px) ** 2 + (my_px - wy_px) ** 2) ** 0.5
    w = max(image_w, 1)
    return (wx_px / w, wy_px / w, size / w)


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


# --- control: clutch-referenced error -> unitless velocity ---------------------------


def _seq3(v):
    """Accept a scalar (broadcast) or a length-3 sequence; return a 3-tuple of floats."""
    if isinstance(v, (int, float)):
        return (float(v), float(v), float(v))
    out = tuple(float(x) for x in v)
    if len(out) != 3:
        raise ValueError(f"expected a scalar or 3 values, got {len(out)}.")
    return out


def _shape3(error_vec, axis_map, gain, deadzone, max_cmd):
    """deadzone + per-axis gain in the INPUT frame, remap to the command frame, clamp.

    Gains/deadzones are given in the intuitive input axes (e.g. image horizontal /
    vertical / depth), then ``axis_map`` reorders+signs into the command frame; clamp is
    symmetric so applying it last is equivalent.
    """
    g = _seq3(gain)
    d = _seq3(deadzone)
    shaped = tuple(retarget.scale(retarget.deadzone(error_vec[i], d[i]), g[i]) for i in range(3))
    remapped = remap(shaped, axis_map)
    return tuple(retarget.clamp(v, -max_cmd, max_cmd) for v in remapped)


def linear_velocity(ref_pos, cur_pos, axis_map, *, gain, deadzone_m, max_cmd):
    """Unitless [-max,max] linear twist from the wrist offset since clutch engage.

    ``gain``/``deadzone_m`` may be a scalar or a length-3 (per input axis) sequence.
    """
    return _shape3(_sub(cur_pos, ref_pos), axis_map, gain, deadzone_m, max_cmd)


def angular_velocity(ref_quat, cur_quat, axis_map, *, gain, deadzone_rad, max_cmd):
    """Unitless [-max,max] angular twist from the orientation offset since engage.

    error = ref^-1 * cur (relative rotation in the reference frame), as a rotation vector.
    The deadzone gates the rotation MAGNITUDE (a rotation vector's components are not
    independent, so a per-component deadzone would be anisotropic), with a soft rescale so
    the command stays continuous at the boundary; then per-axis gain, remap, and clamp.
    ``deadzone_rad`` is a scalar (radians); ``gain`` may be scalar or length-3.
    """
    err = quat_mul(quat_conj(ref_quat), cur_quat)
    rotvec = quat_to_rotvec(err)
    n = math.sqrt(rotvec[0] ** 2 + rotvec[1] ** 2 + rotvec[2] ** 2)
    if n <= deadzone_rad:
        return (0.0, 0.0, 0.0)
    soft = (n - deadzone_rad) / n  # continuous at the deadzone boundary
    g = _seq3(gain)
    shaped = tuple(retarget.scale(rotvec[i] * soft, g[i]) for i in range(3))
    remapped = remap(shaped, axis_map)
    return tuple(retarget.clamp(v, -max_cmd, max_cmd) for v in remapped)


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
