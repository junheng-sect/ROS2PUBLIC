import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'land_v2'

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
    description='Precision landing package with XY/Z/Yaw alignment and BODY_NED yaw_rate control',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'land_v2_node = land_v2.land_v2_node:main',
            'land_v2_csv_logger_node = land_v2.land_v2_csv_logger_node:main',
        ],
    },
)
