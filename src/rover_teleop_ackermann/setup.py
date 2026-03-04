from setuptools import find_packages, setup

package_name = 'rover_teleop_ackermann'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rover_teleop_ackermann.launch.py']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjh',
    maintainer_email='junhengsect+1647307223@qq.com',
    description='Arrow-key teleop and kinematic Ackermann controller for Gazebo rover',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_arrow_teleop_node = '
            'rover_teleop_ackermann.keyboard_arrow_teleop_node:main',
            'ackermann_kinematic_controller_node = '
            'rover_teleop_ackermann.ackermann_kinematic_controller_node:main',
        ],
    },
)
