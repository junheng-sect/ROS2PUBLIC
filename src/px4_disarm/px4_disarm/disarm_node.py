#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PX4 Disarm 服务节点

功能：
- 提供 /disarm 服务
- 调用 MAVROS /mavros/cmd/arming 服务进行 disarm
"""

import rclpy
from rclpy.node import Node
from px4_interfaces.srv import Disarm
from mavros_msgs.srv import CommandBool


class DisarmNode(Node):
    def __init__(self):
        super().__init__('px4_disarm')

        # 创建服务
        self.disarm_server = self.create_service(
            Disarm, 'disarm', self.disarm_callback
        )

        # 创建 MAVROS arming 客户端
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming'
        )

        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')

        self.get_logger().info('PX4 Disarm Node Initialized')

    def disarm_callback(self, request, response):
        """Disarm 回调"""
        try:
            req = CommandBool.Request()
            req.value = False  # False = disarm

            future = self.arming_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            result = future.result()
            if result is not None and result.success:
                response.success = True
                response.message = "Disarm successful"
                self.get_logger().info('Disarm command sent successfully')
            else:
                response.success = False
                response.message = "Disarm failed: no response from FCU"
                self.get_logger().warn('Disarm failed: no response from FCU')

        except Exception as e:
            response.success = False
            response.message = f"Disarm failed: {str(e)}"
            self.get_logger().error(f'Disarm error: {e}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DisarmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
