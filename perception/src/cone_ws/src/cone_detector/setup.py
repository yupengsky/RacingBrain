from setuptools import setup

package_name = "cone_detector"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="todo@example.com",
    description="TODO: camera cone detector skeleton",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        # TODO: 添加 console_scripts，例如:
        # "console_scripts": ["yolo_detector = cone_detector.yolo_detector:main"],
    },
)
