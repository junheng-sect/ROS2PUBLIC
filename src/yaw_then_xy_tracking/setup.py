import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'yaw_then_xy_tracking'

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
    maintainer_email='junhengsect+1647307223@qq.com',
    description='Two-phase ArUco tracking: yaw first, then XY',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yaw_then_xy_tracking_node = yaw_then_xy_tracking.yaw_then_xy_tracking_node:main',
            'yaw_then_xy_tracking_csv_logger_node = yaw_then_xy_tracking.yaw_then_xy_tracking_csv_logger_node:main',
        ],
    },
)
