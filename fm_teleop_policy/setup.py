from glob import glob

from setuptools import find_packages, setup

package_name = "fm_teleop_policy"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/robot_profiles", glob("robot_profiles/*.yaml")),
        # fm-sim catalog: each marker's content is its entry yaml's share path.
        (
            "share/ament_index/resource_index/fm_sim.controllers",
            glob("resource/fm_sim.controllers/*"),
        ),
        (
            "share/" + package_name + "/catalog/controllers",
            glob("catalog/controllers/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="First Motive",
    maintainer_email="nish@ubundi.co.za",
    description="Teleop source: a trained policy drives the arm through the command contract",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "policy_server = fm_teleop_policy.server:main",
        ],
    },
)
