from setuptools import find_packages, setup

package_name = 'simple_pid_controller'

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
    description='Python fallback PID controller for the RacingBrain control stack.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simple_PID_controller_node = simple_pid_controller.simple_PID_controller_node:main',
        ],
    },
)
