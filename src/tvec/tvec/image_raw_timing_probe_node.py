#!/usr/bin/env python3

import threading
import time
from typing import Optional

import rclpy
from debug_interface.msg import ImageRawTiming
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image


class ImageRawTimingProbeNode(Node):
    """提取 /image_raw 的 DDS 时间元信息，用于继续拆分 tvec_queue_sec."""

    def __init__(self):
        super().__init__('image_raw_timing_probe_node')

        # image_topic：需要探测的原始图像话题。
        # image_qos_reliability：与 usb_cam 保持一致，避免 QoS 不匹配导致拿不到样本。
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('image_qos_reliability', 'best_effort')
        self.declare_parameter('timing_topic', '/debug/image_raw_timing')

        self.image_topic = str(self.get_parameter('image_topic').value).strip()
        self.image_qos_reliability = str(
            self.get_parameter('image_qos_reliability').value
        ).strip().lower()
        self.timing_topic = str(self.get_parameter('timing_topic').value).strip()

        if self.image_qos_reliability == 'best_effort':
            reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            reliability = ReliabilityPolicy.RELIABLE
        qos = QoSProfile(reliability=reliability, depth=10)

        # 这里仍创建订阅对象，但不依赖 rclpy 默认执行器回调取消息。
        # 原因是默认执行器只会把消息本体交给回调，拿不到 DDS source_timestamp。
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._noop_callback,
            qos,
        )
        self.timing_pub = self.create_publisher(ImageRawTiming, self.timing_topic, 10)

        self._stop_event = threading.Event()
        self._wait_thread = threading.Thread(
            target=self._probe_loop,
            name='image_raw_timing_probe',
            daemon=True,
        )
        self._wait_thread.start()

        self.get_logger().info(
            'image_raw_timing_probe_node 已启动 | '
            f'image_topic={self.image_topic} | '
            f'image_qos={self.image_qos_reliability} | '
            f'timing_topic={self.timing_topic}'
        )

    def _noop_callback(self, _msg: Image):
        # 本节点不依赖执行器回调处理消息；真正取消息在 _probe_loop 中完成。
        pass

    @staticmethod
    def _ns_to_time_msg(nanoseconds: Optional[int]):
        """把 DDS 返回的纳秒时间戳转成 builtin Time，缺失时返回零时间。"""
        if nanoseconds is None:
            return Time().to_msg()
        try:
            ns_int = int(nanoseconds)
        except (TypeError, ValueError):
            return Time().to_msg()
        if ns_int <= 0:
            return Time().to_msg()
        return Time(nanoseconds=ns_int).to_msg()

    def _probe_loop(self):
        """轮询底层订阅句柄，直接取消息与元信息，避免默认执行器丢弃 source_timestamp。"""
        while not self._stop_event.is_set() and rclpy.ok():
            took_message = False
            while not self._stop_event.is_set():
                with self.image_sub.handle:
                    msg_and_info = self.image_sub.handle.take_message(
                        self.image_sub.msg_type,
                        self.image_sub.raw,
                    )
                if msg_and_info is None:
                    break
                took_message = True

                msg = msg_and_info[0]
                metadata = msg_and_info[1] if len(msg_and_info) >= 2 else {}

                timing_msg = ImageRawTiming()
                timing_msg.header = msg.header
                timing_msg.image_source_stamp = self._ns_to_time_msg(
                    metadata.get('source_timestamp')
                )
                timing_msg.image_received_stamp = self._ns_to_time_msg(
                    metadata.get('received_timestamp')
                )
                timing_msg.image_probe_pub_stamp = self.get_clock().now().to_msg()
                try:
                    self.timing_pub.publish(timing_msg)
                except Exception as exc:
                    if self._stop_event.is_set() or not rclpy.ok():
                        break
                    self.get_logger().error(f'publish image timing failed: {exc}')
                    break

            if not took_message:
                time.sleep(0.001)

    def destroy_node(self):
        self._stop_event.set()
        if hasattr(self, '_wait_thread') and self._wait_thread.is_alive():
            self._wait_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageRawTimingProbeNode()
    try:
        while rclpy.ok():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
