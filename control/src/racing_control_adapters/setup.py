from setuptools import find_packages, setup

package_name = "racing_control_adapters"

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
    maintainer="yupeng",
    maintainer_email="yupeng@todo.todo",
    description="Adapters that feed RacingBrain mapping outputs into the planning-control stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "racingbrain_map_odom_adapter = racing_control_adapters.racingbrain_map_odom_adapter:main",
            "fsds_track_to_map_adapter = racing_control_adapters.fsds_track_to_map_adapter:main",
        ],
    },
)
