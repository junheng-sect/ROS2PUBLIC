import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'altitude_hold'

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
    description=(
        'Range-sensor altitude hold controller with '
        'MAVROS motor signal logging'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'altitude_hold_node = altitude_hold.altitude_hold_node:main',
            (
                'altitude_hold_csv_logger_node = '
                'altitude_hold.altitude_hold_csv_logger_node:main'
            ),
        ],
    },
)
