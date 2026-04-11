import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pid_tuning_v4'

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
            glob(os.path.join('launch', '*.launch.py')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjh',
    maintainer_email='junhengsect+1647307223@qq.com',
    description='PID tuning package derived from pid_tuning_v3 with distance sensor Z hold',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_tuning_v4_node = pid_tuning_v4.pid_tuning_v4_node:main',
            'pid_tuning_v4_csv_logger_node = pid_tuning_v4.pid_tuning_v4_csv_logger_node:main',
        ],
    },
)
