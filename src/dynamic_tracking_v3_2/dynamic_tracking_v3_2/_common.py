#!/usr/bin/env python3
"""dynamic_tracking_v3_2 共享工具函数，供控制节点与 CSV logger 共用。"""

import math


def wrap_to_pi(angle_rad: float) -> float:
    """将角度归一化到 [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def parameter_as_bool(value) -> bool:
    """兼容 launch 字符串和原生 bool 参数，避免 'false' 被 bool() 判真."""
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def rotate_marker_error_to_body(
    ex_marker: float,
    ey_marker: float,
    yaw_rel_corrected: float,
) -> tuple[float, float]:
    """将 marker 平面误差旋转到机体平面误差.

    这里严格采用：
    e_body = R(-yaw_rel_corrected) * e_marker

    其中：
    - ex_marker = target_x - pose.x
    - ey_marker = pose.y - target_y
    - yaw_rel_corrected = 控制使用的相对 yaw + 固定补偿角
    """
    cos_yaw = math.cos(yaw_rel_corrected)
    sin_yaw = math.sin(yaw_rel_corrected)
    ex_body = cos_yaw * ex_marker + sin_yaw * ey_marker
    ey_body = -sin_yaw * ex_marker + cos_yaw * ey_marker
    return ex_body, ey_body
