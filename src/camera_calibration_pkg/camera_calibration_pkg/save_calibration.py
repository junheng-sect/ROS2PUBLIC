#!/usr/bin/env python3
"""
保存标定结果到文件
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml
import os


class SaveCalibrationNode(Node):
    def __init__(self):
        super().__init__('save_calibration')
        
        self.declare_parameter('camera_name', 'default_cam')
        self.declare_parameter('save_path', '/home/zjh/.ros/camera_info')
        
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value
        
        # 订阅标定后的 CameraInfo
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera_info',  # ✅ 正确话题名
            self.camera_info_callback,
            10
        )
        
        self.calibration_saved = False
        self.get_logger().info('Save Calibration Node started')
    
    def camera_info_callback(self, msg):
        if self.calibration_saved:
            return
        
        # 检查是否有标定数据
        if len(msg.k) == 0 or len(msg.d) == 0:
            return
        
        # 创建保存目录
        os.makedirs(self.save_path, exist_ok=True)
        
        # 保存 YAML 文件
        filename = os.path.join(self.save_path, f'{self.camera_name}.yaml')
        
        calibration_data = {
            'image_width': msg.width,
            'image_height': msg.height,
            'camera_name': self.camera_name,
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': list(msg.k)
            },
            'distortion_model': msg.distortion_model,
            'distortion_coefficients': {
                'rows': 1,
                'cols': len(msg.d),
                'data': list(msg.d)
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': list(msg.r)
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': list(msg.p)
            }
        }
        
        with open(filename, 'w') as f:
            yaml.dump(calibration_data, f)
        
        self.get_logger().info(f'✅ Calibration saved to: {filename}')
        self.get_logger().info(f'Camera Matrix: {msg.k}')
        self.get_logger().info(f'Distortion Coeffs: {msg.d}')
        
        self.calibration_saved = True


def main(args=None):
    rclpy.init(args=args)
    node = SaveCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()