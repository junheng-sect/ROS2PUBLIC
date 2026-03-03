#!/usr/bin/env python3
"""
ArUco Gazebo 系统 Launch 文件（带 RViz2 TF 可视化）
- ArUco 标记作为静止坐标系
- 相机/无人机作为动态坐标系
- 🔥 已移除 rqt_image_view，仅使用 RViz2 显示图像
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # ========== 配置区域 ==========
    gz_image_topic = '/world/baylands/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
    ros_image_topic = '/camera/image_raw'
    
    camera_fx = 269.5
    camera_fy = 269.5
    camera_cx = 320.0
    camera_cy = 240.0
    
    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rviz_config_path = os.path.join(pkg_path, 'config', 'aruco_rviz_config.rviz')
    # ================================================================
    
    return LaunchDescription([
        # 1. ROS-GZ 桥接
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_image',
            output='screen',
            arguments=[
                f'{gz_image_topic}@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            remappings=[
                (gz_image_topic, ros_image_topic)
            ],
        ),

        # 2. ArUco 检测节点
        Node(
            package='aruco_gazebo_rviz',
            executable='aruco_gazebo_node',
            name='aruco_gazebo_rviz',
            output='screen',
            parameters=[{
                'image_topic': ros_image_topic,
                'use_compressed': False,
                'publish_annotated_image': True,
                'camera_fx': camera_fx,
                'camera_fy': camera_fy,
                'camera_cx': camera_cx,
                'camera_cy': camera_cy,
                'marker_size_33': 0.135,
                'marker_size_42': 0.017,
                'switch_distance': 0.5,
                'switch_hysteresis': 0.05,
                'enable_csv_log': True,
                'csv_log_path': os.path.expanduser(
                    '~/project/project_ws/src/aruco_gazebo_rviz/aruco_log/gazebo_detection_log.csv'
                ),
            }],
        ),

        # 3. RViz2（TF 可视化 + 图像显示）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_aruco',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])