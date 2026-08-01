import os
from glob import glob
from setuptools import find_packages, setup

package_name = "coverage_frontier_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyyaml", "numpy", "scipy", "pillow"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Collaborative coverage-frontier exploration with dynamic room reallocation for multi-robot ArUco marker detection.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "coverage_grid_server = coverage_frontier_pkg.coverage_grid_server:main",
            "door_reservation_server = coverage_frontier_pkg.door_reservation_server:main",
            "robot_coverage_node = coverage_frontier_pkg.robot_coverage_node:main",
            "coordinator_node = coverage_frontier_pkg.coordinator_node:main",
        ],
    },
)
