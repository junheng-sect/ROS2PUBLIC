from setuptools import find_packages, setup

package_name = 'dynamic_tracking_v3_2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/dynamic_tracking_v3_2.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjh',
    maintainer_email='zjh@example.com',
    description='dynamic_tracking_v3 改进版',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_tracking_v3_2_node = dynamic_tracking_v3_2.dynamic_tracking_v3_node:main',
            'dynamic_tracking_v3_2_csv_logger_node = dynamic_tracking_v3_2.dynamic_tracking_v3_2_csv_logger_node:main',
        ],
    },
)
