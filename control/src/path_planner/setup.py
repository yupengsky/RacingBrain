from setuptools import find_packages, setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crp',
    maintainer_email='2754710939@qq.com',
    description='Formula Student racing path planner for autocross, acceleration, and skidpad missions.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner_node = path_planner.planner_node:main',
        ],
    },
)
