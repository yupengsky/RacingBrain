from setuptools import find_packages, setup


package_name = "cone_detector"


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
    maintainer_email="159146102+yupengsky@users.noreply.github.com",
    description="ROS 2 camera cone detector based on Ultralytics YOLO.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_detector = cone_detector.yolo_detector:main",
        ],
    },
)
