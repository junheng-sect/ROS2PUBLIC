#!/usr/bin/env python3
"""
Gazebo 图像测试 Launch 文件
- 启动 ros_gz_bridge 将 Gazebo 相机图像转为 ROS 话题
- 启动 rqt_image_view 显示图像
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ========== 配置区域 ==========
    # Gazebo 图像话题（根据您的实际环境）
    gz_image_topic = '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
    
    # ROS 话题名称
    ros_image_topic = '/camera/image_raw'
    # ============================
    
    return LaunchDescription([
        # ========== 1. 启动 ROS-GZ 桥接节点 ==========
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_image_test',
            output='screen',
            arguments=[
                # 格式：<gz_topic>@<ros_msg_type>@<gz_msg_type>
                # 方向：Gazebo → ROS
                f'{gz_image_topic}@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            remappings=[
                # 将桥接输出的 ROS 话题重映射为标准话题
                (gz_image_topic, ros_image_topic)
            ],
        ),

        # ========== 2. 启动 rqt_image_view ==========
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view_test',
            output='screen',
            arguments=['--force-discover'],
            remappings=[
                # 订阅桥接后的图像话题
                ('/image_raw', ros_image_topic),
            ],
        ),
    ])