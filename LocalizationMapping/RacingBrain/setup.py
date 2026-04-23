import os
from glob import glob

from setuptools import find_packages, setup

package_name = "racingbrain"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yupeng",
    maintainer_email="yupeng@todo.todo",
    description="RacingBrain real-time localization and mapping orchestration package.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "racingbrain = racingbrain.cli:main",
            "system_health_monitor = racingbrain.health.monitor:main",
        ],
    },
)
