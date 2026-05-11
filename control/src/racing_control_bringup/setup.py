from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'racing_control_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/rviz_config.rviz', 'config/config_autocross.yaml',
                                               'config/config_skidpad.yaml', 'config/config_acceleration.yaml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crp',
    maintainer_email='2754710939@qq.com',
    description='Launch and configuration package for RacingBrain planning and control.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
