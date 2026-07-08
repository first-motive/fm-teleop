from glob import glob

from setuptools import find_packages, setup

package_name = "fm_teleop_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # MediaPipe .task models if fetched (scripts/download_model.sh); empty glob until
        # then, so the build never fails on a missing model. Installed so hand_tracker can
        # resolve them from the package share dir.
        ("share/" + package_name + "/models", glob("models/*.task")),
        # The web control GUI moved to the fm_viewer package (fm-app), which installs it
        # as the system-wide panel viewer. This package keeps only the vision nodes it
        # feeds over the foxglove_bridge websocket.
    ],
    # mediapipe + opencv are pip-only (no rosdep key); install them into the runtime
    # image to run the node. The pure-Python tests (retarget, filter) and the mocked
    # node smoke test do not import them, so colcon test stays green without them.
    install_requires=["setuptools", "mediapipe", "opencv-python", "numpy"],
    zip_safe=True,
    maintainer="First Motive",
    maintainer_email="nish@ubundi.co.za",
    description="Teleop source: vision wrist-tracking (MediaPipe) -> arm twist",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_source = fm_teleop_vision.vision_source:main",
            # 1:1 hand-mirroring path (alongside the wrist-jog vision_source):
            "hand_tracker = fm_teleop_vision.hand_tracker:main",
            "mirror_source = fm_teleop_vision.mirror_source:main",
            # Session recorder (rosbag + synced CSV/JSONL, /capture/record toggle) and the
            # RESET-button re-home node (/vision/reset -> disengage + drive the arm home).
            "mirror_datalogger = fm_teleop_vision.mirror_datalogger:main",
            "arm_reset = fm_teleop_vision.arm_reset:main",
            # Serves recorded sessions (index/detail) to the web GUI's recordings viewer.
            "capture_browser = fm_teleop_vision.capture_browser:main",
        ],
    },
)
