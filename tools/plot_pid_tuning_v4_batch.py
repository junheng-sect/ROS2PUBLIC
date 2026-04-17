#!/usr/bin/env python3
"""批量分析 pid_tuning_v4 CSV 并生成图表。"""

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import matplotlib

# 终端环境直接输出 PNG，避免依赖交互式图形后端。
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch


@dataclass
class SummaryRow:
    """保存现成 summary CSV 中与绘图分析相关的关键字段。"""

    csv_name: str
    run_csv: str
    status: str
    offboard_rows: int
    eval_rows: int
    z_eval_rows: int
    fresh_ratio: float
    distance_valid_ratio: float
    max_stale_s: float
    rmse_x: float
    rmse_y: float
    rmse_z: float
    rmse_xy: float
    p95_abs_x: float
    p95_abs_y: float
    p95_abs_z: float
    p95_xy: float
    cmd_jitter_x: float
    cmd_jitter_y: float
    cmd_jitter_xy: float
    max_abs_yaw_rate: float
    distance_sensor_topic: str
    target_x: float
    target_y: float
    target_z: float
    kp_xy: float
    ki_xy: float
    kd_xy: float
    kp_x: float
    ki_x: float
    kd_x: float
    kp_y: float
    ki_y: float
    kd_y: float
    kp_z: float
    ki_z: float
    kd_z: float
    camera_yaw_compensation_deg: float
    vxy_limit: float
    vx_limit: float
    vy_limit: float
    vz_limit: float
    velocity_deadband: float
    control_rate_hz: float
    pose_timeout_sec: float
    distance_sensor_timeout_sec: float
    require_offboard: int
    enable_z_hold: int


def parse_float(value: str) -> float:
    """将 CSV 文本安全转换为浮点数。"""
    if value is None:
        return math.nan
    text = str(value).strip()
    if not text or text.lower() == "nan":
        return math.nan
    try:
        return float(text)
    except ValueError:
        return math.nan


def parse_int(value: str) -> int:
    """将 CSV 文本安全转换为整数。"""
    number = parse_float(value)
    if math.isnan(number):
        return 0
    return int(number)


def ensure_directory(path: str) -> None:
    """目录不存在时自动创建。"""
    os.makedirs(path, exist_ok=True)


def mask_segments(mask: np.ndarray, time_sec: np.ndarray) -> List[Tuple[float, float]]:
    """把布尔掩码转成若干时间区间，用于阶段加底色。"""
    segments: List[Tuple[float, float]] = []
    start_index = None
    for index, enabled in enumerate(mask):
        if enabled and start_index is None:
            start_index = index
        elif not enabled and start_index is not None:
            segments.append((time_sec[start_index], time_sec[index - 1]))
            start_index = None
    if start_index is not None:
        segments.append((time_sec[start_index], time_sec[-1]))
    return segments


def add_shading(
    axes: Sequence[plt.Axes],
    segments: Sequence[Tuple[float, float]],
    color: str,
    alpha: float,
) -> None:
    """在一组子图上统一标记某些时间区段。"""
    for axis in axes:
        for start_sec, end_sec in segments:
            axis.axvspan(start_sec, end_sec, color=color, alpha=alpha, lw=0)


def safe_min_max(values: np.ndarray) -> Tuple[float, float]:
    """忽略 nan 后获取范围。"""
    finite_values = values[np.isfinite(values)]
    if finite_values.size == 0:
        return -1.0, 1.0
    return float(np.min(finite_values)), float(np.max(finite_values))


def load_summary(summary_csv_path: str, csv_dir: str) -> Dict[str, SummaryRow]:
    """读取现成 summary，并仅保留当前目录里的 CSV 对应记录。"""
    csv_names = {
        file_name
        for file_name in os.listdir(csv_dir)
        if file_name.endswith(".csv") and file_name != os.path.basename(summary_csv_path)
    }

    summary_by_name: Dict[str, SummaryRow] = {}
    with open(summary_csv_path, newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            csv_name = os.path.basename(row["run_csv"])
            if csv_name not in csv_names:
                continue
            summary_by_name[csv_name] = SummaryRow(
                csv_name=csv_name,
                run_csv=row["run_csv"],
                status=row["status"],
                offboard_rows=parse_int(row["offboard_rows"]),
                eval_rows=parse_int(row["eval_rows"]),
                z_eval_rows=parse_int(row["z_eval_rows"]),
                fresh_ratio=parse_float(row["fresh_ratio"]),
                distance_valid_ratio=parse_float(row["distance_valid_ratio"]),
                max_stale_s=parse_float(row["max_stale_s"]),
                rmse_x=parse_float(row["rmse_x"]),
                rmse_y=parse_float(row["rmse_y"]),
                rmse_z=parse_float(row["rmse_z"]),
                rmse_xy=parse_float(row["rmse_xy"]),
                p95_abs_x=parse_float(row["p95_abs_x"]),
                p95_abs_y=parse_float(row["p95_abs_y"]),
                p95_abs_z=parse_float(row["p95_abs_z"]),
                p95_xy=parse_float(row["p95_xy"]),
                cmd_jitter_x=parse_float(row["cmd_jitter_x"]),
                cmd_jitter_y=parse_float(row["cmd_jitter_y"]),
                cmd_jitter_xy=parse_float(row["cmd_jitter_xy"]),
                max_abs_yaw_rate=parse_float(row["max_abs_yaw_rate"]),
                distance_sensor_topic=row["distance_sensor_topic"],
                target_x=parse_float(row["target_x"]),
                target_y=parse_float(row["target_y"]),
                target_z=parse_float(row["target_z"]),
                kp_xy=parse_float(row["kp_xy"]),
                ki_xy=parse_float(row["ki_xy"]),
                kd_xy=parse_float(row["kd_xy"]),
                kp_x=parse_float(row["kp_x"]),
                ki_x=parse_float(row["ki_x"]),
                kd_x=parse_float(row["kd_x"]),
                kp_y=parse_float(row["kp_y"]),
                ki_y=parse_float(row["ki_y"]),
                kd_y=parse_float(row["kd_y"]),
                kp_z=parse_float(row["kp_z"]),
                ki_z=parse_float(row["ki_z"]),
                kd_z=parse_float(row["kd_z"]),
                camera_yaw_compensation_deg=parse_float(row["camera_yaw_compensation_deg"]),
                vxy_limit=parse_float(row["vxy_limit"]),
                vx_limit=parse_float(row["vx_limit"]),
                vy_limit=parse_float(row["vy_limit"]),
                vz_limit=parse_float(row["vz_limit"]),
                velocity_deadband=parse_float(row["velocity_deadband"]),
                control_rate_hz=parse_float(row["control_rate_hz"]),
                pose_timeout_sec=parse_float(row["pose_timeout_sec"]),
                distance_sensor_timeout_sec=parse_float(row["distance_sensor_timeout_sec"]),
                require_offboard=parse_int(row["require_offboard"]),
                enable_z_hold=parse_int(row["enable_z_hold"]),
            )
    return summary_by_name


def load_run_csv(csv_path: str) -> Dict[str, np.ndarray]:
    """读取单次 CSV 为 numpy 数组。"""
    with open(csv_path, newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise ValueError(f"CSV 为空: {csv_path}")

    first_time = parse_float(rows[0]["ros_time_sec"])
    numeric_fields = [
        "aruco_fresh",
        "aruco_age_sec",
        "aruco_x",
        "aruco_y",
        "aruco_z",
        "distance_sensor_z",
        "distance_sensor_fresh",
        "distance_sensor_age_sec",
        "distance_sensor_valid",
        "ez_from_vision",
        "ez_from_distance_sensor",
        "aruco_yaw_rad",
        "yaw_rel_raw_rad",
        "yaw_rel_corrected_rad",
        "ex_marker",
        "ey_marker",
        "ex_body",
        "ey_body",
        "local_pose_fresh",
        "local_pose_age_sec",
        "local_x",
        "local_y",
        "local_z",
        "local_yaw_rad",
        "setpoint_fresh",
        "setpoint_age_sec",
        "sp_frame",
        "sp_type_mask",
        "sp_vx",
        "sp_vy",
        "sp_vz",
        "sp_yaw_rate",
        "target_x",
        "target_y",
        "target_z",
        "kp_xy",
        "ki_xy",
        "kd_xy",
        "kp_x",
        "ki_x",
        "kd_x",
        "kp_y",
        "ki_y",
        "kd_y",
        "kp_z",
        "ki_z",
        "kd_z",
        "camera_yaw_compensation_deg",
        "vxy_limit",
        "vx_limit",
        "vy_limit",
        "vz_limit",
        "velocity_deadband",
        "control_rate_hz",
        "pose_timeout_sec",
        "distance_sensor_timeout_sec",
        "require_offboard",
        "enable_z_hold",
    ]

    data: Dict[str, List[float]] = {"time_sec": []}
    for field_name in numeric_fields:
        data[field_name] = []

    modes: List[str] = []
    z_source_used: List[str] = []

    for row in rows:
        data["time_sec"].append(parse_float(row["ros_time_sec"]) - first_time)
        for field_name in numeric_fields:
            data[field_name].append(parse_float(row[field_name]))
        modes.append(row["mode"].strip())
        z_source_used.append(row["z_source_used"].strip())

    array_data = {key: np.asarray(value, dtype=float) for key, value in data.items()}
    array_data["mode"] = np.asarray(modes, dtype=object)
    array_data["z_source_used"] = np.asarray(z_source_used, dtype=object)
    return array_data


def replay_vision_vz(data: Dict[str, np.ndarray], summary: SummaryRow) -> np.ndarray:
    """按 v4 节点 PID 规则，用视觉高度误差重放一条假想 vz 输出。"""
    time_sec = data["time_sec"]
    replayed_vz = np.full(time_sec.shape, 0.0, dtype=float)

    kp_z = summary.kp_z if math.isfinite(summary.kp_z) else 0.8
    ki_z = summary.ki_z if math.isfinite(summary.ki_z) else 0.0
    kd_z = summary.kd_z if math.isfinite(summary.kd_z) else 0.06
    vz_limit = summary.vz_limit if math.isfinite(summary.vz_limit) else 0.5
    velocity_deadband = summary.velocity_deadband if math.isfinite(summary.velocity_deadband) else 0.03
    default_dt = 1.0 / max(summary.control_rate_hz, 1.0) if math.isfinite(summary.control_rate_hz) else 1.0 / 30.0
    i_limit = 1.0

    integral = 0.0
    prev_error = 0.0
    prev_time = None

    for index, current_time in enumerate(time_sec):
        mode_ok = data["mode"][index] == "OFFBOARD"
        z_hold_enabled = int(data["enable_z_hold"][index]) == 1
        vision_fresh = int(data["aruco_fresh"][index]) == 1
        error = data["ez_from_vision"][index]
        error_finite = math.isfinite(error)

        if (not mode_ok) or (not z_hold_enabled) or (not vision_fresh) or (not error_finite):
            integral = 0.0
            prev_error = 0.0
            prev_time = None
            replayed_vz[index] = 0.0
            continue

        if prev_time is None:
            dt = default_dt
        else:
            dt = current_time - prev_time
            if dt <= 1e-6:
                dt = 1e-2

        # 保持与节点实现一致的 PID 结构。
        p_term = kp_z * error
        integral += error * dt
        integral = max(-i_limit, min(i_limit, integral))
        i_term = ki_z * integral
        d_term = kd_z * (error - prev_error) / dt

        vz = p_term + i_term + d_term
        vz = max(-vz_limit, min(vz_limit, vz))
        if abs(vz) < velocity_deadband:
            vz = 0.0

        replayed_vz[index] = vz
        prev_error = error
        prev_time = current_time

    return replayed_vz


def save_aruco_timeseries_plot(run_name: str, output_dir: str, data: Dict[str, np.ndarray], summary: SummaryRow) -> None:
    """生成 ArUco 状态随时间变化图。"""
    time_sec = data["time_sec"]
    offboard_segments = mask_segments(data["mode"] == "OFFBOARD", time_sec)

    figure, axes = plt.subplots(4, 1, figsize=(16, 12), sharex=True)
    figure.suptitle("Aruco Tracking States vs Time", fontsize=15)
    add_shading(axes, offboard_segments, "#f4c38b", 0.25)

    plot_specs = [
        ("aruco_x", "target_x", "aruco_x (m)", "aruco_x", "target_x"),
        ("aruco_y", "target_y", "aruco_y (m)", "aruco_y", "target_y"),
        ("aruco_z", "target_z", "aruco_z (m)", "aruco_z", "target_z"),
        ("aruco_yaw_rad", None, "aruco_yaw_rad (rad)", "aruco_yaw_rad", "target_yaw"),
    ]
    for axis, (state_key, target_key, y_label, state_label, target_label) in zip(axes, plot_specs):
        axis.plot(time_sec, data[state_key], color="#3b8dd5", linewidth=1.6, label=state_label)
        if target_key is None:
            axis.axhline(0.0, color="#ff6b6b", linestyle="--", linewidth=1.0, label=target_label)
        else:
            axis.plot(time_sec, data[target_key], color="#ff6b6b", linestyle="--", linewidth=1.0, label=target_label)
        axis.grid(True, linestyle="--", alpha=0.3)
        axis.set_ylabel(y_label)
        axis.legend(loc="upper right")

    axes[-1].set_xlabel("time_sec (s)")
    if not math.isnan(summary.rmse_xy):
        figure.text(
            0.99,
            0.01,
            f"status={summary.status}, rmse_xy={summary.rmse_xy:.3f} m, rmse_z={summary.rmse_z:.3f} m",
            ha="right",
            va="bottom",
            fontsize=10,
            color="#444444",
        )
    figure.tight_layout(rect=(0, 0.02, 1, 0.97))
    figure.savefig(os.path.join(output_dir, f"{run_name}_plot2_aruco_timeseries.png"), dpi=160, bbox_inches="tight")
    plt.close(figure)


def save_setpoint_timeseries_plot(run_name: str, output_dir: str, data: Dict[str, np.ndarray], summary: SummaryRow) -> None:
    """生成 PositionTarget 速度指令随时间变化图。"""
    time_sec = data["time_sec"]
    offboard_segments = mask_segments(data["mode"] == "OFFBOARD", time_sec)

    vx_limit = summary.vx_limit if math.isfinite(summary.vx_limit) else summary.vxy_limit
    vy_limit = summary.vy_limit if math.isfinite(summary.vy_limit) else summary.vxy_limit
    vz_limit = summary.vz_limit if math.isfinite(summary.vz_limit) else 0.5

    figure, axes = plt.subplots(4, 1, figsize=(16, 12), sharex=True)
    figure.suptitle("Setpoint Velocity Commands vs Time", fontsize=15)
    add_shading(axes, offboard_segments, "#f4c38b", 0.25)

    plot_specs = [
        ("sp_vx", "sp_vx", "sp_vx (m/s)", vx_limit, "vx_limit"),
        ("sp_vy", "sp_vy", "sp_vy (m/s)", vy_limit, "vy_limit"),
        ("sp_vz", "sp_vz", "sp_vz (m/s)", vz_limit, "vz_limit"),
        ("sp_yaw_rate", "sp_yaw_rate", "sp_yaw_rate (rad/s)", 1.0, "yaw_rate_limit"),
    ]
    for axis, (data_key, label, y_label, limit_value, limit_label) in zip(axes, plot_specs):
        axis.plot(time_sec, data[data_key], color="#3b8dd5", linewidth=1.5, label=label)
        axis.axhline(limit_value, color="#ff6b6b", linestyle="--", linewidth=1.0, label=f"+{limit_label}")
        axis.axhline(-limit_value, color="#ff6b6b", linestyle="--", linewidth=1.0, label=f"-{limit_label}")
        axis.grid(True, linestyle="--", alpha=0.3)
        axis.set_ylabel(y_label)
        axis.legend(loc="upper right")

    axes[-1].set_xlabel("time_sec (s)")
    figure.tight_layout(rect=(0, 0, 1, 0.97))
    figure.savefig(os.path.join(output_dir, f"{run_name}_plot3_setpoint_timeseries.png"), dpi=160, bbox_inches="tight")
    plt.close(figure)


def save_xy_trajectory_plot(run_name: str, output_dir: str, data: Dict[str, np.ndarray]) -> None:
    """生成 OFFBOARD 阶段的 XY 轨迹图。"""
    valid_mask = (
        (data["mode"] == "OFFBOARD")
        & np.isfinite(data["aruco_x"])
        & np.isfinite(data["aruco_y"])
    )
    figure, axis = plt.subplots(figsize=(9, 9))
    axis.set_title("Aruco XY Tracking Trajectory (OFFBOARD Only)")
    axis.set_xlabel("aruco_x (m)")
    axis.set_ylabel("aruco_y (m)")
    axis.grid(True, linestyle="--", alpha=0.3)
    axis.set_aspect("equal", adjustable="box")

    if np.count_nonzero(valid_mask) >= 2:
        x_values = data["aruco_x"][valid_mask]
        y_values = data["aruco_y"][valid_mask]
        t_values = data["time_sec"][valid_mask]
        axis.plot(x_values, y_values, color="#6b7280", alpha=0.25, linewidth=1.0)
        scatter = axis.scatter(x_values, y_values, c=t_values, cmap="viridis", s=18, edgecolors="none", label="aruco trajectory")
        axis.scatter(data["target_x"][valid_mask][-1], data["target_y"][valid_mask][-1], marker="x", color="#ef4444", s=90, linewidths=2, label="target")
        axis.scatter(x_values[0], y_values[0], marker="o", color="#2f9e44", s=80, label="start")
        axis.scatter(x_values[-1], y_values[-1], marker="s", color="#1971c2", s=70, label="end")
        color_bar = figure.colorbar(scatter, ax=axis)
        color_bar.set_label("time_sec (s)")
        axis.legend(loc="upper right")
    else:
        axis.text(0.5, 0.5, "No valid OFFBOARD XY trajectory data", ha="center", va="center", fontsize=14, transform=axis.transAxes)

    min_x, max_x = safe_min_max(data["aruco_x"])
    min_y, max_y = safe_min_max(data["aruco_y"])
    axis.set_xlim(min_x - 0.1, max_x + 0.1)
    axis.set_ylim(min_y - 0.1, max_y + 0.1)
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, f"{run_name}_plot4_aruco_xy_trajectory_offboard_only.png"), dpi=160, bbox_inches="tight")
    plt.close(figure)


def save_velocity_vector_plot(
    run_name: str,
    output_dir: str,
    data: Dict[str, np.ndarray],
    sample_step: int = 10,
) -> Dict[str, float]:
    """在 XY 轨迹图上绘制速度矢量，并统计其是否朝向 target。"""
    valid_mask = (
        (data["mode"] == "OFFBOARD")
        & np.isfinite(data["aruco_x"])
        & np.isfinite(data["aruco_y"])
        & np.isfinite(data["sp_vx"])
        & np.isfinite(data["sp_vy"])
        & np.isfinite(data["yaw_rel_corrected_rad"])
    )
    valid_indices = np.flatnonzero(valid_mask)
    sampled_indices = valid_indices[::sample_step]

    figure, axis = plt.subplots(figsize=(10, 10))
    axis.set_title("Velocity Vectors Toward Target (OFFBOARD, every 10 samples)")
    axis.set_xlabel("aruco_x (m)")
    axis.set_ylabel("aruco_y (m)")
    axis.grid(True, linestyle="--", alpha=0.3)
    axis.set_aspect("equal", adjustable="box")

    stats = {
        "sample_count": float(sampled_indices.size),
        "aligned_count": 0.0,
        "misaligned_count": 0.0,
        "alignment_ratio": math.nan,
        "mean_cosine": math.nan,
    }
    if sampled_indices.size == 0:
        axis.text(0.5, 0.5, "No valid OFFBOARD samples with velocity commands", ha="center", va="center", fontsize=14, transform=axis.transAxes)
        figure.tight_layout()
        figure.savefig(os.path.join(output_dir, f"{run_name}_plot5_velocity_vectors_to_target.png"), dpi=160, bbox_inches="tight")
        plt.close(figure)
        return stats

    all_x = data["aruco_x"][valid_indices]
    all_y = data["aruco_y"][valid_indices]
    sampled_x = data["aruco_x"][sampled_indices]
    sampled_y = data["aruco_y"][sampled_indices]
    sampled_sp_vx = data["sp_vx"][sampled_indices]
    sampled_sp_vy = data["sp_vy"][sampled_indices]
    sampled_tx = data["target_x"][sampled_indices]
    sampled_ty = data["target_y"][sampled_indices]
    sampled_yaw = data["yaw_rel_corrected_rad"][sampled_indices]

    # 先从 BODY_NED 消息值恢复为内部 FLU 机体系速度。
    vx_body = sampled_sp_vx
    vy_body = -sampled_sp_vy

    # 再把 body 速度映射回 marker 误差坐标系。
    cos_yaw = np.cos(sampled_yaw)
    sin_yaw = np.sin(sampled_yaw)
    vx_marker_like = cos_yaw * vx_body - sin_yaw * vy_body
    vy_marker_like = sin_yaw * vx_body + cos_yaw * vy_body

    # 图上的 y 轴是 aruco_y，而 ey_marker = aruco_y - target_y，因此再翻转一次 y。
    vx_plot = vx_marker_like
    vy_plot = -vy_marker_like

    to_target_x = sampled_tx - sampled_x
    to_target_y = sampled_ty - sampled_y
    cmd_norm = np.hypot(vx_plot, vy_plot)
    target_norm = np.hypot(to_target_x, to_target_y)
    valid_alignment = (cmd_norm > 1e-6) & (target_norm > 1e-6)

    cosine_values = np.full(sampled_indices.shape, np.nan, dtype=float)
    cosine_values[valid_alignment] = (
        vx_plot[valid_alignment] * to_target_x[valid_alignment]
        + vy_plot[valid_alignment] * to_target_y[valid_alignment]
    ) / (cmd_norm[valid_alignment] * target_norm[valid_alignment])

    aligned_mask = cosine_values > 0.0
    misaligned_mask = cosine_values < 0.0
    stats["aligned_count"] = float(np.count_nonzero(aligned_mask))
    stats["misaligned_count"] = float(np.count_nonzero(misaligned_mask))
    if np.count_nonzero(valid_alignment) > 0:
        stats["alignment_ratio"] = float(np.count_nonzero(aligned_mask) / np.count_nonzero(valid_alignment))
        stats["mean_cosine"] = float(np.nanmean(cosine_values))

    axis.plot(all_x, all_y, color="#9ca3af", linewidth=1.0, alpha=0.5, label="aruco trajectory")
    axis.scatter(sampled_x, sampled_y, s=14, color="#111827", alpha=0.65, label="sampled points")

    scale = 1.8
    if np.any(aligned_mask):
        axis.quiver(
            sampled_x[aligned_mask],
            sampled_y[aligned_mask],
            vx_plot[aligned_mask],
            vy_plot[aligned_mask],
            angles="xy",
            scale_units="xy",
            scale=scale,
            color="#2f9e44",
            width=0.004,
            alpha=0.9,
            label="sp aligned to target",
        )
    if np.any(misaligned_mask):
        axis.quiver(
            sampled_x[misaligned_mask],
            sampled_y[misaligned_mask],
            vx_plot[misaligned_mask],
            vy_plot[misaligned_mask],
            angles="xy",
            scale_units="xy",
            scale=scale,
            color="#e03131",
            width=0.004,
            alpha=0.9,
            label="sp away from target",
        )

    axis.scatter(sampled_tx[-1], sampled_ty[-1], marker="x", color="#ef4444", s=100, linewidths=2, label="target")
    axis.scatter(sampled_x[0], sampled_y[0], marker="o", color="#2f9e44", s=75, label="start")
    axis.scatter(sampled_x[-1], sampled_y[-1], marker="s", color="#1971c2", s=70, label="end")

    text_lines = [
        f"sampled={int(stats['sample_count'])}",
        f"aligned={int(stats['aligned_count'])}",
        f"misaligned={int(stats['misaligned_count'])}",
    ]
    if not math.isnan(stats["alignment_ratio"]):
        text_lines.append(f"alignment_ratio={stats['alignment_ratio']:.3f}")
    if not math.isnan(stats["mean_cosine"]):
        text_lines.append(f"mean_cosine={stats['mean_cosine']:.3f}")
    axis.text(
        0.02,
        0.98,
        "\n".join(text_lines),
        transform=axis.transAxes,
        ha="left",
        va="top",
        fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85, "edgecolor": "#d0d0d0"},
    )

    min_x, max_x = safe_min_max(data["aruco_x"])
    min_y, max_y = safe_min_max(data["aruco_y"])
    axis.set_xlim(min_x - 0.1, max_x + 0.1)
    axis.set_ylim(min_y - 0.1, max_y + 0.1)
    axis.legend(loc="upper right")
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, f"{run_name}_plot5_velocity_vectors_to_target.png"), dpi=160, bbox_inches="tight")
    plt.close(figure)
    return stats


def save_z_control_comparison_plot(
    run_name: str,
    output_dir: str,
    data: Dict[str, np.ndarray],
    summary: SummaryRow,
) -> Dict[str, float]:
    """新增高度控制来源对比图：拆分显示视觉重放 vz 与测距仪实际 sp_vz。"""
    time_sec = data["time_sec"]
    offboard_mask = data["mode"] == "OFFBOARD"
    offboard_segments = mask_segments(offboard_mask, time_sec)
    distance_invalid_mask = offboard_mask & (data["z_source_used"] != "distance_sensor")
    distance_invalid_segments = mask_segments(distance_invalid_mask, time_sec)

    vz_from_vision_replay = replay_vision_vz(data, summary)

    figure, axes = plt.subplots(4, 1, figsize=(16, 15), sharex=True)
    figure.suptitle("Z Control Source Comparison", fontsize=15)
    add_shading(axes, offboard_segments, "#f4c38b", 0.20)
    add_shading(axes, distance_invalid_segments, "#ef9a9a", 0.18)

    axes[0].plot(time_sec, data["aruco_z"], color="#3b8dd5", linewidth=1.4, label="aruco_z")
    axes[0].plot(time_sec, data["distance_sensor_z"], color="#2f9e44", linewidth=1.4, label="distance_sensor_z")
    axes[0].plot(time_sec, data["target_z"], color="#ff6b6b", linestyle="--", linewidth=1.0, label="target_z")
    axes[0].set_ylabel("height (m)")
    axes[0].grid(True, linestyle="--", alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(time_sec, data["ez_from_vision"], color="#3b8dd5", linewidth=1.4, label="ez_from_vision")
    axes[1].plot(time_sec, data["ez_from_distance_sensor"], color="#2f9e44", linewidth=1.4, label="ez_from_distance_sensor")
    axes[1].axhline(0.0, color="#777777", linestyle="--", linewidth=0.8)
    axes[1].set_ylabel("error (m)")
    axes[1].grid(True, linestyle="--", alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(time_sec, vz_from_vision_replay, color="#6c43ce", linewidth=1.4, label="vz_from_vision_replay")
    axes[2].axhline(summary.vz_limit, color="#ff6b6b", linestyle="--", linewidth=1.0, label="+vz_limit")
    axes[2].axhline(-summary.vz_limit, color="#ff6b6b", linestyle="--", linewidth=1.0, label="-vz_limit")
    axes[2].set_ylabel("vision vz (m/s)")
    axes[2].grid(True, linestyle="--", alpha=0.3)
    axes[2].legend(loc="upper right")

    axes[3].plot(time_sec, data["sp_vz"], color="#ff8c42", linewidth=1.4, label="sp_vz_actual(distance_sensor)")
    axes[3].axhline(summary.vz_limit, color="#ff6b6b", linestyle="--", linewidth=1.0, label="+vz_limit")
    axes[3].axhline(-summary.vz_limit, color="#ff6b6b", linestyle="--", linewidth=1.0, label="-vz_limit")
    axes[3].set_ylabel("distance vz (m/s)")
    axes[3].set_xlabel("time_sec (s)")
    axes[3].grid(True, linestyle="--", alpha=0.3)

    legend_handles, legend_labels = axes[3].get_legend_handles_labels()
    legend_handles.extend(
        [
            Patch(facecolor="#f4c38b", alpha=0.20, label="OFFBOARD"),
            Patch(facecolor="#ef9a9a", alpha=0.18, label="distance invalid/unused"),
        ]
    )
    legend_labels.extend(["OFFBOARD", "distance invalid/unused"])
    axes[3].legend(legend_handles, legend_labels, loc="upper right")

    compare_mask = offboard_mask & np.isfinite(vz_from_vision_replay) & np.isfinite(data["sp_vz"])
    if np.count_nonzero(compare_mask) > 0:
        vz_diff = vz_from_vision_replay[compare_mask] - data["sp_vz"][compare_mask]
        compare_stats = {
            "vz_replay_peak_abs": float(np.nanmax(np.abs(vz_from_vision_replay[compare_mask]))),
            "vz_actual_peak_abs": float(np.nanmax(np.abs(data["sp_vz"][compare_mask]))),
            "vz_abs_diff_mean": float(np.nanmean(np.abs(vz_diff))),
        }
        figure.text(
            0.99,
            0.01,
            " | ".join(
                [
                    f"distance_valid_ratio={summary.distance_valid_ratio:.3f}" if math.isfinite(summary.distance_valid_ratio) else "distance_valid_ratio=nan",
                    f"mean|vision_vz - actual_sp_vz|={compare_stats['vz_abs_diff_mean']:.3f} m/s",
                ]
            ),
            ha="right",
            va="bottom",
            fontsize=10,
            color="#444444",
        )
    else:
        compare_stats = {
            "vz_replay_peak_abs": math.nan,
            "vz_actual_peak_abs": math.nan,
            "vz_abs_diff_mean": math.nan,
        }

    figure.tight_layout(rect=(0, 0.02, 1, 0.97))
    figure.savefig(os.path.join(output_dir, f"{run_name}_plot6_z_control_source_comparison.png"), dpi=160, bbox_inches="tight")
    plt.close(figure)
    return compare_stats


def write_run_summary(
    csv_name: str,
    output_dir: str,
    data: Dict[str, np.ndarray],
    summary: SummaryRow,
    velocity_vector_stats: Dict[str, float],
    z_compare_stats: Dict[str, float],
) -> None:
    """写出单次实验文本摘要。"""
    time_sec = data["time_sec"]
    offboard_mask = data["mode"] == "OFFBOARD"
    offboard_duration_sec = 0.0
    if np.count_nonzero(offboard_mask) >= 2:
        offboard_duration_sec = float(time_sec[offboard_mask][-1] - time_sec[offboard_mask][0])

    lines = [
        f"csv_path: {csv_name}",
        f"status: {summary.status}",
        f"sample_count: {len(time_sec)}",
        f"duration_sec: {time_sec[-1]:.3f}",
        f"offboard_duration_sec: {offboard_duration_sec:.3f}",
        f"offboard_rows: {summary.offboard_rows}",
        f"eval_rows: {summary.eval_rows}",
        f"z_eval_rows: {summary.z_eval_rows}",
        f"fresh_ratio: {summary.fresh_ratio:.3f}" if math.isfinite(summary.fresh_ratio) else "fresh_ratio: nan",
        f"distance_valid_ratio: {summary.distance_valid_ratio:.3f}" if math.isfinite(summary.distance_valid_ratio) else "distance_valid_ratio: nan",
        f"max_stale_s: {summary.max_stale_s:.3f}" if math.isfinite(summary.max_stale_s) else "max_stale_s: nan",
        f"rmse_xy: {summary.rmse_xy:.3f}" if math.isfinite(summary.rmse_xy) else "rmse_xy: nan",
        f"rmse_z: {summary.rmse_z:.3f}" if math.isfinite(summary.rmse_z) else "rmse_z: nan",
        f"p95_xy: {summary.p95_xy:.3f}" if math.isfinite(summary.p95_xy) else "p95_xy: nan",
        f"target_x: {summary.target_x:.3f}" if math.isfinite(summary.target_x) else "target_x: nan",
        f"target_y: {summary.target_y:.3f}" if math.isfinite(summary.target_y) else "target_y: nan",
        f"target_z: {summary.target_z:.3f}" if math.isfinite(summary.target_z) else "target_z: nan",
        f"distance_sensor_topic: {summary.distance_sensor_topic}",
        f"camera_yaw_compensation_deg: {summary.camera_yaw_compensation_deg:.3f}" if math.isfinite(summary.camera_yaw_compensation_deg) else "camera_yaw_compensation_deg: nan",
        f"kp_z: {summary.kp_z:.3f}" if math.isfinite(summary.kp_z) else "kp_z: nan",
        f"vz_limit: {summary.vz_limit:.3f}" if math.isfinite(summary.vz_limit) else "vz_limit: nan",
        f"max_abs_yaw_rate: {summary.max_abs_yaw_rate:.3f}" if math.isfinite(summary.max_abs_yaw_rate) else "max_abs_yaw_rate: nan",
        f"velocity_vector_sample_count: {int(velocity_vector_stats['sample_count'])}",
        f"velocity_vector_aligned_count: {int(velocity_vector_stats['aligned_count'])}",
        f"velocity_vector_misaligned_count: {int(velocity_vector_stats['misaligned_count'])}",
        (
            f"velocity_vector_alignment_ratio: {velocity_vector_stats['alignment_ratio']:.3f}"
            if math.isfinite(velocity_vector_stats["alignment_ratio"])
            else "velocity_vector_alignment_ratio: nan"
        ),
        (
            f"velocity_vector_mean_cosine: {velocity_vector_stats['mean_cosine']:.3f}"
            if math.isfinite(velocity_vector_stats["mean_cosine"])
            else "velocity_vector_mean_cosine: nan"
        ),
        (
            f"vz_from_vision_replay_peak_abs: {z_compare_stats['vz_replay_peak_abs']:.3f}"
            if math.isfinite(z_compare_stats["vz_replay_peak_abs"])
            else "vz_from_vision_replay_peak_abs: nan"
        ),
        (
            f"sp_vz_actual_peak_abs: {z_compare_stats['vz_actual_peak_abs']:.3f}"
            if math.isfinite(z_compare_stats["vz_actual_peak_abs"])
            else "sp_vz_actual_peak_abs: nan"
        ),
        (
            f"mean_abs_diff_vision_vs_actual_vz: {z_compare_stats['vz_abs_diff_mean']:.3f}"
            if math.isfinite(z_compare_stats["vz_abs_diff_mean"])
            else "mean_abs_diff_vision_vs_actual_vz: nan"
        ),
    ]
    with open(os.path.join(output_dir, f"{os.path.splitext(csv_name)[0]}_plot_summary.txt"), "w", encoding="utf-8") as file:
        file.write("\n".join(lines) + "\n")


def build_overall_report(summary_rows: List[SummaryRow], output_path: str, batch_name: str) -> None:
    """输出本批实验总体分析文件。"""
    rows = sorted(summary_rows, key=lambda item: item.csv_name)
    ok_rows = [row for row in rows if row.status == "ok"]
    insufficient_rows = [row.csv_name for row in rows if row.status != "ok"]
    ranked_by_xy = sorted(ok_rows, key=lambda item: (math.isnan(item.rmse_xy), item.rmse_xy))

    lines = [
        f"# {batch_name} pid_tuning_v4 数据分析",
        "",
        f"- 总实验数：{len(rows)}",
        f"- 有效实验数（status=ok）：{len(ok_rows)}",
        f"- 无效实验数（insufficient_data 等）：{len(insufficient_rows)}",
        "",
        "## RMSE_XY 从优到差",
    ]
    for row in ranked_by_xy:
        lines.append(
            f"- {row.csv_name}: rmse_xy={row.rmse_xy:.3f} m, rmse_z={row.rmse_z:.3f} m, "
            f"fresh_ratio={row.fresh_ratio:.3f}, distance_valid_ratio={row.distance_valid_ratio:.3f}"
        )

    lines.append("")
    lines.append("## 无效实验")
    if insufficient_rows:
        for csv_name in insufficient_rows:
            lines.append(f"- {csv_name}")
    else:
        lines.append("- 无")

    with open(output_path, "w", encoding="utf-8") as file:
        file.write("\n".join(lines) + "\n")


def main() -> None:
    """程序入口。"""
    parser = argparse.ArgumentParser(description="批量分析 pid_tuning_v4 CSV 并生成图片")
    parser.add_argument("--csv-dir", required=True, help="待分析 CSV 目录")
    parser.add_argument("--summary-csv", required=True, help="pid_tuning_v4_summary.csv 路径")
    parser.add_argument("--output-dir", required=True, help="输出 plots 根目录")
    args = parser.parse_args()

    ensure_directory(args.output_dir)
    summary_by_name = load_summary(args.summary_csv, args.csv_dir)

    csv_names = sorted(
        file_name
        for file_name in os.listdir(args.csv_dir)
        if file_name.endswith(".csv") and file_name != os.path.basename(args.summary_csv)
    )

    summary_rows: List[SummaryRow] = []
    for csv_name in csv_names:
        if csv_name not in summary_by_name:
            raise KeyError(f"summary 中缺少统计结果: {csv_name}")

        csv_path = os.path.join(args.csv_dir, csv_name)
        run_name = os.path.splitext(csv_name)[0]
        run_output_dir = os.path.join(args.output_dir, run_name)
        ensure_directory(run_output_dir)

        data = load_run_csv(csv_path)
        summary = summary_by_name[csv_name]
        summary_rows.append(summary)

        save_aruco_timeseries_plot(run_name, run_output_dir, data, summary)
        save_setpoint_timeseries_plot(run_name, run_output_dir, data, summary)
        save_xy_trajectory_plot(run_name, run_output_dir, data)
        velocity_vector_stats = save_velocity_vector_plot(run_name, run_output_dir, data)
        z_compare_stats = save_z_control_comparison_plot(run_name, run_output_dir, data, summary)
        write_run_summary(csv_name, run_output_dir, data, summary, velocity_vector_stats, z_compare_stats)
        print(f"generated: {run_output_dir}")

    batch_name = os.path.basename(os.path.normpath(args.csv_dir))
    build_overall_report(summary_rows, os.path.join(args.output_dir, "analysis_summary.md"), batch_name)
    print(f"generated: {os.path.join(args.output_dir, 'analysis_summary.md')}")


if __name__ == "__main__":
    main()
