from setuptools import find_packages, setup
import os
from glob import glob

# 🔥 关键：包名必须是 aruco_gazebo
package_name = 'aruco_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjh',
    maintainer_email='zjh@todo.todo',
    description='ArUco Detector for Gazebo Simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 🔥 关键：executable 名称
            'aruco_gazebo_node = aruco_gazebo.aruco_node:main',
        ],
    },
)