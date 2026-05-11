import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dynamic_tracking_v3'

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
        'Dynamic tracking v3 package derived from dynamic_tracking_v2 '
        'with Z/yaw smoothing'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dynamic_tracking_v3_node = '
            'dynamic_tracking_v3.dynamic_tracking_v3_node:main',
            'dynamic_tracking_v3_csv_logger_node = '
            'dynamic_tracking_v3.dynamic_tracking_v3_csv_logger_node:main',
        ],
    },
)
