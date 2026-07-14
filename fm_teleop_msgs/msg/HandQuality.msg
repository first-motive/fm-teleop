# Compact per-frame tracking-quality signal for one hand — a lightweight channel a
# dataset consumer can subscribe to (or the recorder can score) without
# deserializing the full HandSkeleton. One message per hand per frame.

# stamp shared with the matching HandSkeleton / camera frame.
std_msgs/Header header

# "left" | "right" (see HandSkeleton.hand). Empty if unknown.
string hand

# True when a hand was tracked this frame.
bool detected

# MediaPipe presence/handedness score in [0, 1].
float32 confidence

# True when every landmark lies inside the image bounds; false = clipped at an edge
# (partial occlusion / hand leaving frame).
bool in_frame

# Residual landmark motion after smoothing (metres) — a temporal-noise proxy. Higher
# = jerkier / less reliable tracking this frame.
float32 jitter
