from setuptools import find_packages, setup
import os
from glob import glob

package_name = "yolo_signals"

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
    maintainer="victor",
    maintainer_email="victor@todo.todo",
    description="YOLOv8-based traffic sign detection node for autonomous navigation experiments.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["signsDetector = yolo_signals.signsDetector:main"],
    },
)
