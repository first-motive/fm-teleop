# fm-teleop image — the teleop layer, FROM the robot layer.
#
# Adds MoveIt + MoveIt Servo on top of the robot layer's control/viz/description
# tooling, so this image can run the Servo teleop pipeline the input nodes feed.
# The robot layer is itself FROM the shared fm-docker base, so the control and
# viz/xacro tooling are inherited rather than rebuilt. The entrypoint and WORKDIR
# are inherited too — this layer only adds apt packages.
FROM ghcr.io/first-motive/fm-robot:humble

ARG DEBIAN_FRONTEND=noninteractive

# MoveIt + MoveIt Servo: the planning stack plus the realtime Cartesian/joint
# jogger the teleop input nodes publish to. Both on the Humble apt mirror for
# arm64 and amd64, so no source builds.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-moveit \
      ros-humble-moveit-servo \
    && rm -rf /var/lib/apt/lists/*
