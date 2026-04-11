import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pid_tuning_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjh',
    maintainer_email='zjh@example.com',
    description='PID tuning v2 package for ArUco tracking with XY/Z control and camera yaw compensation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_tuning_v2_node = pid_tuning_v2.pid_tuning_node:main',
            'pid_tuning_v2_csv_logger_node = pid_tuning_v2.pid_tuning_csv_logger_node:main',
        ],
    },
)
