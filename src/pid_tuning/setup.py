import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pid_tuning'

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
    description='PID tuning package for ArUco tracking with XY split tuning and metrics summary',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_tuning_node = pid_tuning.pid_tuning_node:main',
            'csv_logger_node = pid_tuning.pid_tuning_csv_logger_node:main',
        ],
    },
)
