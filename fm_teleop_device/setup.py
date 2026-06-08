from setuptools import find_packages, setup

package_name = "fm_teleop_device"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="First Motive",
    maintainer_email="nish@ubundi.co.za",
    description="Teleop sources for physical devices: gamepad, SpaceMouse, and the G1-D hand mapper",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_to_servo = fm_teleop_device.joy_to_servo:main",
            "spacenav_to_servo = fm_teleop_device.spacenav_to_servo:main",
            "g1_hand_teleop = fm_teleop_device.g1_hand_teleop:main",
        ],
    },
)
