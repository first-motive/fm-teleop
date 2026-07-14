# Full human-hand skeleton for one hand in one video frame — the "second data
# stream" derived from MediaPipe Hands and recorded alongside the raw egocentric
# video. Everything MediaPipe produces per frame, kept (not collapsed to a wrist
# pose): 21 landmarks in both 3D metres and 2D pixels, palm orientation, per-finger
# joint angles, and the grip proxy. Landmark index order is the MediaPipe Hands
# 21-point topology (see fm_teleop_vision/hand.py LANDMARK_NAMES):
#   0 wrist
#   1 thumb_cmc   2 thumb_mcp   3 thumb_ip    4 thumb_tip
#   5 index_mcp   6 index_pip   7 index_dip   8 index_tip
#   9 middle_mcp 10 middle_pip 11 middle_dip 12 middle_tip
#  13 ring_mcp   14 ring_pip   15 ring_dip   16 ring_tip
#  17 pinky_mcp  18 pinky_pip  19 pinky_dip  20 pinky_tip

# stamp = frame capture time (shared with the camera frame it was computed from);
# frame_id = the camera frame the landmarks are expressed in.
std_msgs/Header header

# "left" | "right", anatomically corrected for the selfie-mirror label swap. Empty
# if handedness is unknown.
string hand

# True when a hand was tracked this frame. When false, the landmark arrays hold the
# last value and consumers should ignore them.
bool detected

# MediaPipe presence/handedness score in [0, 1].
float32 confidence

# 21 landmarks in world metres, wrist-origin (3D). Index order = topology above.
geometry_msgs/Point[21] world_landmarks

# 21 landmarks in image pixels (x = px, y = py, z = 0). Index order = topology above.
geometry_msgs/Point[21] image_landmarks

# Orientation of the palm frame (x = wrist->middle knuckle, z = palm normal).
geometry_msgs/Quaternion palm_orientation

# Per-finger articulation in radians. Flexion = angle between adjacent bones
# (0 = straight, larger = more bent). Fixed length 19, layout:
#    0- 2  thumb   [cmc, mcp, ip]
#    3- 5  index   [mcp, pip, dip]
#    6- 8  middle  [mcp, pip, dip]
#    9-11  ring    [mcp, pip, dip]
#   12-14  pinky   [mcp, pip, dip]
#   15-18  abduction [thumb-index, index-middle, middle-ring, ring-pinky]
float32[] joint_angles

# Finger-curl grip proxy in [0, 1] (0 = open, 1 = closed) — the same signal used to
# drive the gripper, carried through for convenience.
float32 grip
