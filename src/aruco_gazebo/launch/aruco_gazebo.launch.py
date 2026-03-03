#!/usr/bin/env python3
"""
ArUco Gazebo 系统 Launch 文件
- 启动 ros_gz_bridge 将 Gazebo 图像话题转为 ROS 话题
- 启动 ArUco 检测节点
- 启动 rqt_image_view 显示标注图像
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # ========== 配置区域（根据成功的 gazebo_image_test 配置）==========
    # Gazebo 图像话题（已验证正确的话题）
    gz_image_topic = '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
    
    # ROS 话题名称（用于可视化和通用命名）
    ros_image_topic = '/camera/image_raw'
    # 说明：当前 parameter_bridge 日志显示其 ROS 侧最终话题名为 gz_image_topic，
    # 因此检测节点直接订阅 bridge 实际输出话题，避免因 remap 未生效导致无图像。
    detector_input_topic = gz_image_topic
    
    # 相机内参（根据实际相机分辨率调整）
    # 如果分辨率是 640x480，hfov=1.74 rad：
    camera_fx = 269.5
    camera_fy = 269.5
    camera_cx = 320.0
    camera_cy = 240.0
    # ================================================================
    
    return LaunchDescription([
        # ========== 1. 启动 ROS-GZ 桥接节点 ==========
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_image',
            output='screen',
            arguments=[
                # 格式：<gz_topic>@<ros_msg_type>@<gz_msg_type>
                f'{gz_image_topic}@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            remappings=[
                # 将桥接输出的 ROS 话题重映射为标准话题
                (gz_image_topic, ros_image_topic)
            ],
        ),

        # ========== 2. 启动 ArUco 检测节点 ==========
        Node(
            package='aruco_gazebo',
            executable='aruco_gazebo_node',
            name='aruco_gazebo',
            output='screen',
            parameters=[{
                'image_topic': detector_input_topic,  # 订阅 bridge 实际输出话题
                'use_compressed': False,         # Gazebo 发布原始图像
                'publish_annotated_image': True,
                
                # 相机内参（根据实际相机调整）
                'camera_fx': camera_fx,
                'camera_fy': camera_fy,
                'camera_cx': camera_cx,
                'camera_cy': camera_cy,
                
                # ArUco 标记参数
                'marker_size_33': 0.135,
                'marker_size_42': 0.017,
                'switch_distance': 0.5,
                'switch_hysteresis': 0.05,
                
                # CSV 日志配置
                'enable_csv_log': True,
                'csv_log_path': os.path.expanduser(
                    '~/project/project_ws/src/aruco_gazebo/aruco_log/gazebo_detection_log.csv'
                ),
            }],
        ),

        # ========== 3. 启动 rqt_image_view（参考成功的配置）==========
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view_gazebo',
            output='screen',
            arguments=['--force-discover'],
            remappings=[
                # 订阅标注后的图像
                ('/image_raw', '/aruco/image_annotated'),
            ],
        ),
    ])
