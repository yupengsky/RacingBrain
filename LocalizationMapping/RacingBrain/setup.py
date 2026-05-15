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
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
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
            "lidar_backend_arbiter = racingbrain.perception.lidar_backend_arbiter:main",
            "lidar_cones_to_map = racingbrain.perception.lidar_cones_to_map:main",
            "lio_gnss_error_eval = racingbrain.localization.lio_gnss_error_eval_node:main",
            "lio_imu_adapter = racingbrain.localization.lio_imu_adapter_node:main",
            "lio_pointcloud_adapter = racingbrain.localization.lio_pointcloud_adapter_node:main",
            "multisource_pose_judge = racingbrain.localization.multisource_pose_judge_node:main",
            "pointpillars_local_verifier = racingbrain.perception.pointpillars_local_verifier:main",
            "track_graph_builder = racingbrain.planning.track_graph_node:main",
        ],
    },
)
