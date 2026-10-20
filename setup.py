from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "tb4_maps"


def optional_files(pattern):
    return [path for path in glob(pattern) if Path(path).is_file()]


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/maps", optional_files("maps/*.yaml") + optional_files("maps/*.pgm")),
        (f"share/{package_name}/config", optional_files("config/*.yaml") + optional_files("config/*.rviz")),
        (f"share/{package_name}/worlds", optional_files("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Student",
    maintainer_email="student@example.com",
    description="TurtleBot 4 simulation, SLAM mapping, AMCL localization, Nav2 goal navigation and YOLO detection.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "goal_navigator = tb4_maps.goal_navigator:main",
            "yolo_detector = tb4_maps.yolo_detector:main",
        ],
    },
)
