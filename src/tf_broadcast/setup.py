from setuptools import find_packages, setup

package_name = 'tf_broadcast'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjh',
    maintainer_email='junhengsect+1647307223@qq.com',
    description='Static TF broadcaster for map to aruco_marker',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_aruco_static_tf_node = tf_broadcast.map_aruco_static_tf_node:main',
            'map_base_link_tf_node = tf_broadcast.map_base_link_tf_node:main',
            'cam_tf_node = tf_broadcast.cam_tf_node:main',
            'vision_pose_tf_node = tf_broadcast.vision_pose_tf_node:main',
        ],
    },
)
