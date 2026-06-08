from setuptools import find_packages, setup

package_name = "fm_teleop_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
        ],
    },
)
