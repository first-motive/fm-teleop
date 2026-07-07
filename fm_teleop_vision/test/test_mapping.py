"""Pure unit tests for fm_teleop_vision.mapping — no ROS graph, no camera, no mediapipe.

Covers the quaternion/axis math, the clutch-referenced position/orientation -> velocity
control, and the gripper hysteresis. (cv2/mediapipe are NOT imported by mapping, so these
run anywhere fm_teleop_core is on the path.)
"""

import math

import pytest

from fm_teleop_vision import mapping


# --- quaternion + geometry -----------------------------------------------------------


def test_identity_quat_has_zero_rotvec():
    assert mapping.quat_to_rotvec(mapping.IDENTITY_QUAT) == (0.0, 0.0, 0.0)


def test_rotvec_of_z_rotation():
    theta = 0.4
    q = (math.cos(theta / 2), 0.0, 0.0, math.sin(theta / 2))  # rotation about +z
    rx, ry, rz = mapping.quat_to_rotvec(q)
    assert rx == pytest.approx(0.0, abs=1e-6)
    assert ry == pytest.approx(0.0, abs=1e-6)
    assert rz == pytest.approx(theta, abs=1e-6)


def test_quat_from_axes_identity():
    q = mapping.quat_from_axes((1, 0, 0), (0, 1, 0), (0, 0, 1))
    assert q[0] == pytest.approx(1.0)
    assert q[1:] == pytest.approx((0.0, 0.0, 0.0))


def test_palm_orientation_degenerate_is_identity():
    p = (0.0, 0.0, 0.0)
    assert mapping.palm_orientation(p, p, p, p) == mapping.IDENTITY_QUAT


def test_palm_orientation_is_unit_quat():
    wrist = (0.0, 0.0, 0.0)
    middle_mcp = (0.0, 0.10, 0.0)   # hand points +y
    index_mcp = (-0.03, 0.09, 0.0)
    pinky_mcp = (0.03, 0.09, 0.0)
    q = mapping.palm_orientation(wrist, index_mcp, middle_mcp, pinky_mcp)
    assert math.sqrt(sum(c * c for c in q)) == pytest.approx(1.0, abs=1e-6)


# --- finger curl ---------------------------------------------------------------------


def _curl(open_factor):
    """Helper: tips at `open_factor` of the hand scale from their MCPs."""
    wrist = (0.0, 0.0, 0.0)
    middle_mcp = (0.0, 0.10, 0.0)       # scale = 0.10
    mcps = [(0.0, 0.10, 0.0)] * 4
    tips = [(0.0, 0.10 + open_factor * 0.10, 0.0)] * 4
    return mapping.finger_curl(wrist, middle_mcp, mcps, tips)


def test_finger_curl_extended_low_curled_high():
    extended = _curl(1.3)   # tips far from mcps -> openness high -> curl low
    curled = _curl(0.2)     # tips near mcps      -> openness low  -> curl high
    assert extended < 0.2
    assert curled > 0.8
    assert 0.0 <= extended <= 1.0 and 0.0 <= curled <= 1.0


def test_finger_curl_degenerate_zero():
    p = (0.0, 0.0, 0.0)
    assert mapping.finger_curl(p, p, [p], [p]) == 0.0


# --- axis map ------------------------------------------------------------------------


def test_parse_and_remap_axis_map():
    parsed = mapping.parse_axis_map(["z", "x", "-y"])
    assert mapping.remap((1.0, 2.0, 3.0), parsed) == (3.0, 1.0, -2.0)


def test_parse_axis_map_validates():
    with pytest.raises(ValueError):
        mapping.parse_axis_map(["x", "y"])          # wrong length
    with pytest.raises(ValueError):
        mapping.parse_axis_map(["x", "y", "w"])     # bad axis


# --- linear velocity (clutch-referenced) ---------------------------------------------

_IDENT = mapping.parse_axis_map(["x", "y", "z"])


def test_linear_zero_at_reference():
    v = mapping.linear_velocity((0, 0, 0), (0, 0, 0), _IDENT, gain=5.0, deadzone_m=0.005, max_cmd=1.0)
    assert v == (0.0, 0.0, 0.0)


def test_linear_deadzone_suppresses_tiny_offset():
    v = mapping.linear_velocity((0, 0, 0), (0.002, 0, 0), _IDENT, gain=5.0, deadzone_m=0.005, max_cmd=1.0)
    assert v == (0.0, 0.0, 0.0)


def test_linear_scales_and_clamps():
    v = mapping.linear_velocity((0, 0, 0), (0.1, 0, 0), _IDENT, gain=5.0, deadzone_m=0.005, max_cmd=1.0)
    assert v[0] == pytest.approx(0.5)            # 0.1 * 5
    big = mapping.linear_velocity((0, 0, 0), (1.0, 0, 0), _IDENT, gain=5.0, deadzone_m=0.005, max_cmd=1.0)
    assert big[0] == pytest.approx(1.0)          # clamped to max_cmd


def test_linear_respects_axis_map():
    amap = mapping.parse_axis_map(["z", "x", "-y"])
    v = mapping.linear_velocity((0, 0, 0), (0.1, 0, 0), amap, gain=1.0, deadzone_m=0.0, max_cmd=1.0)
    assert v == pytest.approx((0.0, 0.1, 0.0))   # input x -> output y


# --- angular velocity ----------------------------------------------------------------


def test_angular_zero_when_unrotated():
    v = mapping.angular_velocity(mapping.IDENTITY_QUAT, mapping.IDENTITY_QUAT, _IDENT,
                                 gain=1.0, deadzone_rad=0.0, max_cmd=10.0)
    assert v == (0.0, 0.0, 0.0)


def test_angular_tracks_relative_rotation():
    theta = 0.3
    cur = (math.cos(theta / 2), 0.0, 0.0, math.sin(theta / 2))
    v = mapping.angular_velocity(mapping.IDENTITY_QUAT, cur, _IDENT,
                                 gain=1.0, deadzone_rad=0.0, max_cmd=10.0)
    assert v[2] == pytest.approx(theta, abs=1e-6)
    assert v[0] == pytest.approx(0.0, abs=1e-6)
    assert v[1] == pytest.approx(0.0, abs=1e-6)


def test_angular_deadzone_is_magnitude_based():
    # Deadzone must gate the rotation MAGNITUDE, not per-axis components: an off-axis
    # rotation just above the threshold must still command motion (the per-component bug
    # would zero it because each component is below the threshold).
    axis = (1.0 / math.sqrt(2), 1.0 / math.sqrt(2), 0.0)

    def quat(theta):
        s = math.sin(theta / 2)
        return (math.cos(theta / 2), axis[0] * s, axis[1] * s, axis[2] * s)

    below = mapping.angular_velocity(mapping.IDENTITY_QUAT, quat(0.04), _IDENT,
                                     gain=1.0, deadzone_rad=0.05, max_cmd=10.0)
    assert below == (0.0, 0.0, 0.0)              # magnitude 0.04 < 0.05 -> suppressed

    above = mapping.angular_velocity(mapping.IDENTITY_QUAT, quat(0.06), _IDENT,
                                     gain=1.0, deadzone_rad=0.05, max_cmd=10.0)
    assert above[0] > 0.0 and above[1] > 0.0     # magnitude 0.06 > 0.05 -> not zeroed
    assert above[2] == pytest.approx(0.0, abs=1e-9)


# --- absolute pose mirroring ---------------------------------------------------------

_BIG_BOX = ((-10.0, -10.0, -10.0), (10.0, 10.0, 10.0))


def test_mirror_target_holds_at_reference():
    # hand_now == hand_ref -> zero delta -> target is exactly ee_ref (no startup jump, holds).
    ee_ref = (0.3, 0.0, 0.4)
    t = mapping.mirror_target(ee_ref, (0.5, 0.5, 0.5), (0.5, 0.5, 0.5), _IDENT,
                              scale=[0.4, 0.4, 0.4], workspace_box=_BIG_BOX)
    assert t == pytest.approx(ee_ref)


def test_mirror_target_applies_metres_scale():
    # +0.1 normalized hand-x, scale 0.4 m/unit, identity map -> +0.04 m on command x.
    t = mapping.mirror_target((0.3, 0.0, 0.4), (0.5, 0.5, 0.5), (0.6, 0.5, 0.5), _IDENT,
                              scale=[0.4, 0.4, 0.0], workspace_box=_BIG_BOX)
    assert t[0] == pytest.approx(0.34)
    assert t[1] == pytest.approx(0.0)
    assert t[2] == pytest.approx(0.4)


def test_mirror_target_respects_axis_map():
    # ["z","x","-y"]: input-x delta routes to command-frame y (matches test_linear_respects_axis_map).
    amap = mapping.parse_axis_map(["z", "x", "-y"])
    t = mapping.mirror_target((0.0, 0.0, 0.0), (0.5, 0.5, 0.5), (0.6, 0.5, 0.5), amap,
                              scale=1.0, workspace_box=_BIG_BOX)
    assert t == pytest.approx((0.0, 0.1, 0.0))


def test_mirror_target_depth_scale_zero_ignores_depth():
    # A pure depth (input-z) move with depth scale 0 must not move the target.
    ee_ref = (0.3, 0.0, 0.4)
    t = mapping.mirror_target(ee_ref, (0.5, 0.5, 0.5), (0.5, 0.5, 0.9), _IDENT,
                              scale=[0.4, 0.4, 0.0], workspace_box=_BIG_BOX)
    assert t == pytest.approx(ee_ref)


def test_mirror_target_clamps_to_workspace_box():
    # A large hand move saturates at the box face; the target never exceeds it.
    box = ((0.1, -0.5, 0.0), (0.5, 0.5, 0.8))
    t = mapping.mirror_target((0.45, 0.0, 0.4), (0.5, 0.5, 0.5), (1.0, 0.5, 0.5), _IDENT,
                              scale=[1.0, 1.0, 0.0], workspace_box=box)
    assert t[0] == pytest.approx(0.5)            # 0.45 + 0.5 = 0.95 -> clamped to xmax 0.5
    assert t[1] == pytest.approx(0.0)
    assert t[2] == pytest.approx(0.4)


# --- metric mirroring (image units -> metres via apparent hand size) ------------------


def test_image_width_m_pinhole():
    # hand spans 6% of the image width and is 0.09 m long -> the frame is 1.5 m wide there.
    assert mapping.image_width_m(0.06, hand_span_m=0.09) == pytest.approx(1.5)


def test_image_width_m_degenerate_is_none():
    assert mapping.image_width_m(0.0, hand_span_m=0.09) is None
    assert mapping.image_width_m(1e-6, hand_span_m=0.09) is None
    assert mapping.image_width_m(-0.05, hand_span_m=0.09) is None
    assert mapping.image_width_m(None, hand_span_m=0.09) is None


def test_metric_scale_planar_and_depth():
    s = mapping.metric_scale(1.5, mirror_gain=1.0, axis_gain=[1.0, 1.0, 0.0], fallback_scale=0.9)
    assert s == pytest.approx((1.5, 1.5, 0.0))
    # depth gain carries NO width factor — depth_delta_m is already metres.
    s = mapping.metric_scale(1.5, mirror_gain=1.0, axis_gain=[1.0, 1.0, 1.0], fallback_scale=0.9)
    assert s[2] == pytest.approx(1.0)
    # mirror_gain scales every axis.
    s = mapping.metric_scale(1.5, mirror_gain=0.5, axis_gain=[1.0, 1.0, 1.0], fallback_scale=0.9)
    assert s == pytest.approx((0.75, 0.75, 0.5))


def test_metric_scale_fallback_when_width_none():
    s = mapping.metric_scale(None, mirror_gain=1.0, axis_gain=[1.0, 1.0, 0.0], fallback_scale=0.9)
    assert s == pytest.approx((0.9, 0.9, 0.0))


def test_depth_delta_toward_camera_positive():
    # span 0.09 m: z 0.09 -> 0.12 means distance 1.0 m -> 0.75 m: the hand came 0.25 m closer.
    assert mapping.depth_delta_m(0.09, 0.12, hand_span_m=0.09) == pytest.approx(0.25)
    # degenerate sizes never inject a spike
    assert mapping.depth_delta_m(0.0, 0.12, hand_span_m=0.09) == 0.0
    assert mapping.depth_delta_m(0.09, 0.0, hand_span_m=0.09) == 0.0


def test_metric_mirror_target_one_to_one():
    # z 0.06 -> the frame is 1.5 m wide; a 0.2 image-width move is 0.30 m of real travel.
    scale = mapping.metric_scale(mapping.image_width_m(0.06, hand_span_m=0.09),
                                 mirror_gain=1.0, axis_gain=[1.0, 1.0, 0.0], fallback_scale=0.9)
    target, overflow = mapping.metric_mirror_target(
        (0.3, 0.0, 0.4), (0.5, 0.5, 0.06), (0.7, 0.5, 0.06), _IDENT,
        scale=scale, hand_span_m=0.09, workspace_box=_BIG_BOX)
    assert target[0] == pytest.approx(0.6)       # 0.3 + 0.30 m: true 1:1
    assert target[1:] == pytest.approx((0.0, 0.4))
    assert overflow == (0.0, 0.0, 0.0)


def test_metric_mirror_target_xy_uniform():
    # Equal image-width deltas in x and y -> equal metres (y is normalized by WIDTH).
    scale = mapping.metric_scale(1.5, mirror_gain=1.0, axis_gain=[1.0, 1.0, 0.0], fallback_scale=0.9)
    target, _ = mapping.metric_mirror_target(
        (0.0, 0.0, 0.0), (0.4, 0.4, 0.06), (0.5, 0.5, 0.06), _IDENT,
        scale=scale, hand_span_m=0.09, workspace_box=_BIG_BOX)
    assert target[0] == pytest.approx(target[1])
    assert target[0] == pytest.approx(0.15)


def test_metric_mirror_target_depth_opt_in():
    # A pure depth move (apparent size grows): axis_gain[2]=0 holds; =1 moves the mapped
    # axis by the metric depth delta (0.25 m, see test_depth_delta_toward_camera_positive).
    ee_ref = (0.3, 0.0, 0.4)
    off = mapping.metric_scale(1.5, mirror_gain=1.0, axis_gain=[1.0, 1.0, 0.0], fallback_scale=0.9)
    target, _ = mapping.metric_mirror_target(
        ee_ref, (0.5, 0.5, 0.09), (0.5, 0.5, 0.12), _IDENT,
        scale=off, hand_span_m=0.09, workspace_box=_BIG_BOX)
    assert target == pytest.approx(ee_ref)
    on = mapping.metric_scale(1.5, mirror_gain=1.0, axis_gain=[1.0, 1.0, 1.0], fallback_scale=0.9)
    target, _ = mapping.metric_mirror_target(
        ee_ref, (0.5, 0.5, 0.09), (0.5, 0.5, 0.12), _IDENT,
        scale=on, hand_span_m=0.09, workspace_box=_BIG_BOX)
    assert target[2] == pytest.approx(0.65)      # identity map: input depth -> command z


def test_metric_mirror_target_overflow_reports_clamp():
    box = ((0.0, -0.5, 0.0), (0.5, 0.5, 0.8))
    scale = mapping.metric_scale(1.5, mirror_gain=1.0, axis_gain=[1.0, 1.0, 0.0], fallback_scale=0.9)
    # raw x = 0.45 + 0.1*1.5 = 0.60 -> clamped to xmax 0.5, +0.10 m overflow reported.
    target, overflow = mapping.metric_mirror_target(
        (0.45, 0.0, 0.4), (0.5, 0.5, 0.06), (0.6, 0.5, 0.06), _IDENT,
        scale=scale, hand_span_m=0.09, workspace_box=box)
    assert target[0] == pytest.approx(0.5)
    assert overflow[0] == pytest.approx(0.10)
    # raw x = 0.45 - 0.6 = -0.15 -> clamped to xmin 0.0, -0.15 m overflow.
    target, overflow = mapping.metric_mirror_target(
        (0.45, 0.0, 0.4), (0.5, 0.5, 0.06), (0.1, 0.5, 0.06), _IDENT,
        scale=scale, hand_span_m=0.09, workspace_box=box)
    assert target[0] == pytest.approx(0.0)
    assert overflow[0] == pytest.approx(-0.15)


def test_box_clamp_inside_is_zero_overflow():
    clamped, overflow = mapping.box_clamp((0.1, 0.2, 0.3), _BIG_BOX)
    assert clamped == pytest.approx((0.1, 0.2, 0.3))
    assert overflow == (0.0, 0.0, 0.0)


def test_step_limit_passthrough_within_budget():
    # A move smaller than the per-tick budget on every axis is published unchanged.
    prev = (0.30, 0.00, 0.40)
    target = (0.31, -0.01, 0.405)
    assert mapping.step_limit(prev, target, 0.02) == pytest.approx(target)


def test_step_limit_clamps_large_multi_axis_jump():
    # A jump beyond the budget is slewed to prev +/- max_step per axis (sign preserved).
    prev = (0.30, 0.00, 0.40)
    target = (0.50, -0.15, 0.30)      # +0.20, -0.15, -0.10 -> all exceed 0.02
    out = mapping.step_limit(prev, target, 0.02)
    assert out == pytest.approx((0.32, -0.02, 0.38))


def test_step_limit_disabled_returns_target():
    prev = (0.30, 0.00, 0.40)
    target = (0.50, -0.15, 0.30)
    # None prev (first tick after engage) and non-positive budget both disable the cap.
    assert mapping.step_limit(None, target, 0.02) == pytest.approx(target)
    assert mapping.step_limit(prev, target, 0.0) == pytest.approx(target)
    assert mapping.step_limit(prev, target, -1.0) == pytest.approx(target)


def test_control_position_normalizes_y_by_width():
    pos = mapping.control_position((960.0, 540.0), (960.0, 640.0), 1920)
    assert pos[0] == pytest.approx(0.5)
    assert pos[1] == pytest.approx(540.0 / 1920.0)   # y over WIDTH: uniform units with x
    assert pos[2] == pytest.approx(100.0 / 1920.0)   # apparent size over width


# --- gripper -------------------------------------------------------------------------


def test_grip_preset_hysteresis():
    assert mapping.grip_preset(0.9, "open", open_below=0.35, close_above=0.65) == "close"
    assert mapping.grip_preset(0.1, "close", open_below=0.35, close_above=0.65) == "open"
    # in the dead band the previous state holds
    assert mapping.grip_preset(0.5, "close", open_below=0.35, close_above=0.65) == "close"
    assert mapping.grip_preset(0.5, "open", open_below=0.35, close_above=0.65) == "open"
    assert mapping.grip_preset(0.5, None) == "open"


def test_grip_position_endpoints():
    assert mapping.grip_position(0.0, open_pos=1.5, close_pos=-0.17) == pytest.approx(1.5)
    assert mapping.grip_position(1.0, open_pos=1.5, close_pos=-0.17) == pytest.approx(-0.17)
