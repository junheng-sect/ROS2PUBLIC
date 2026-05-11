#!/usr/bin/env python3
"""
摄像头标定 Launch 文件
启动摄像头 + 标定节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 参数声明
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/image_raw',
        description='摄像头话题名称'
    )

    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='USB 摄像头设备路径'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='图像宽度'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='图像高度'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='15.0',
        description='摄像头帧率'
    )

    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='mjpeg2rgb',
        description='像素格式，常用 mjpeg2rgb 或 yuyv2rgb'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='default_cam',
        description='相机名称，用于区分不同相机或不同分辨率的标定结果'
    )

    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='file:///home/zjh/.ros/camera_info/default_cam.yaml',
        description='标定文件保存地址，使用 file:/// 形式避免覆盖旧标定'
    )
    
    size_arg = DeclareLaunchArgument(
        'size',
        default_value='7x5',
        description='标定板内角点数（列 x 行）'
    )
    
    square_arg = DeclareLaunchArgument(
        'square',
        default_value='0.041',
        description='棋盘格每个方格的边长（米）'
    )

    calib_queue_size_arg = DeclareLaunchArgument(
        'calib_queue_size',
        default_value='1',
        description='标定节点图像队列大小，建议保持 1 降低内存占用'
    )

    max_chessboard_speed_arg = DeclareLaunchArgument(
        'max_chessboard_speed',
        default_value='0.5',
        description='采样速度上限（px/frame），用于抑制快速运动造成的无效样本堆积'
    )

    return LaunchDescription([
        camera_topic_arg,
        video_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        pixel_format_arg,
        camera_name_arg,
        camera_info_url_arg,
        size_arg,
        square_arg,
        calib_queue_size_arg,
        max_chessboard_speed_arg,

        # 1. 摄像头驱动
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'framerate': LaunchConfiguration('framerate'),
                'camera_frame_id': 'camera_link',
                # 允许为不同相机/分辨率保存独立标定文件，避免覆盖默认标定。
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }],
        ),

        # 2. 标定节点
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='camera_calibrator',
            output='screen',
            parameters=[{
                'size': LaunchConfiguration('size'),
                'square': LaunchConfiguration('square'),
            }],
            remappings=[
                # 单目标定只需要图像与 camera_info 两个输入话题。
                ('image', LaunchConfiguration('camera_topic')),
                ('camera_info', '/camera_info'),
                # cameracalibrator 内部固定访问 `camera/set_camera_info`。
                # usb_cam 实际提供的服务名是 `/set_camera_info`，不是 `/usb_cam/set_camera_info`。
                # 这里显式重映射服务名，避免点击 COMMIT 后同步 RPC 永久阻塞。
                ('camera/set_camera_info', '/set_camera_info'),
            ],
            arguments=[
                '--size', LaunchConfiguration('size'),
                '--square', LaunchConfiguration('square'),
                '--camera_name', LaunchConfiguration('camera_name'),
                '--queue-size', LaunchConfiguration('calib_queue_size'),
                '--max-chessboard-speed', LaunchConfiguration('max_chessboard_speed'),
            ],
        ),
    ])
