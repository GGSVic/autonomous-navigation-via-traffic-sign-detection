from setuptools import find_packages, setup
import os
from glob import glob

package_name = "autonomous_navigation"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="puzzlebot",
    maintainer_email="puzzlebot@todo.todo",
    description="ROS 2 package implementing lane following and autonomous navigation behaviors using classical computer vision and traffic sign detections. ",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_resizer = autonomous_navigation.image_resizer:main",
            "lineFollower = autonomous_navigation.lineFollower:main",
            "controller = autonomous_navigation.controller:main",
        ],
    },
)
