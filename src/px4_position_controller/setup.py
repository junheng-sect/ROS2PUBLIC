from setuptools import setup

package_name = 'px4_position_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjh',
    maintainer_email='zjh@example.com',
    description='PX4 Offboard Position Controller using PID and MAVROS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_controller = px4_position_controller.position_controller_node:main'
        ],
    },
)