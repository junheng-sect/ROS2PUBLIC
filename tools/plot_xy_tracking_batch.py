#!/usr/bin/env python3
"""批量分析 xy_tracking CSV 并生成与历史模板一致的图表。"""

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import matplotlib

# 在无图形桌面的终端环境中直接输出 PNG，避免依赖交互式后端。
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


@dataclass
class SummaryRow:
    """保存 summary CSV 中与分析相关的关键字段。"""

    run_csv: str
    status: str
    offboard_rows: int
    eval_rows: int
    fresh_ratio: float
    max_stale_s: float
    rmse_x: float
    rmse_y: float
    rmse_z: float
    rmse_xy: float
    p95_xy: float
    kp_xy: float
    kd_xy: float
    kp_z: float
    vz_limit: float
    vxy_limit: float


def parse_float(value: str) -> float:
    """将 CSV 字符串安全转换为浮点数，非法值统一转为 nan。"""
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
    """将 CSV 字符串安全转换为整数，非法值回落为 0。"""
    number = parse_float(value)
    if math.isnan(number):
        return 0
    return int(number)


def load_summary(summary_csv_path: str, csv_dir: str) -> Dict[str, SummaryRow]:
    """读取 summary，并仅保留当前目录内 CSV 对应的统计结果。"""
    csv_names = {
        file_name
        for file_name in os.listdir(csv_dir)
        if file_name.endswith(".csv")
    }
    summary_by_name: Dict[str, SummaryRow] = {}
    with open(summary_csv_path, newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            csv_name = os.path.basename(row["run_csv"])
            if csv_name not in csv_names:
                continue
            summary_by_name[csv_name] = SummaryRow(
                run_csv=row["run_csv"],
                status=row["status"],
                offboard_rows=parse_int(row["offboard_rows"]),
                eval_rows=parse_int(row["eval_rows"]),
                fresh_ratio=parse_float(row["fresh_ratio"]),
                max_stale_s=parse_float(row["max_stale_s"]),
                rmse_x=parse_float(row["rmse_x"]),
                rmse_y=parse_float(row["rmse_y"]),
                rmse_z=parse_float(row["rmse_z"]),
                rmse_xy=parse_float(row["rmse_xy"]),
                p95_xy=parse_float(row["p95_xy"]),
                kp_xy=parse_float(row["kp_xy"]),
                kd_xy=parse_float(row["kd_xy"]),
                kp_z=parse_float(row["kp_z"]),
                vz_limit=parse_float(row["vz_limit"]),
                vxy_limit=parse_float(row["vxy_limit"]),
            )
    return summary_by_name


def load_run_csv(csv_path: str) -> Dict[str, np.ndarray]:
    """将单次实验 CSV 转为 numpy 数组，便于后续批量画图和统计。"""
    with open(csv_path, newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise ValueError(f"CSV 为空: {csv_path}")

    first_time = parse_float(rows[0]["ros_time_sec"])
    data: Dict[str, List[float]] = {
        "time_sec": [],
        "aruco_x": [],
        "aruco_y": [],
        "aruco_z": [],
        "aruco_yaw_rad": [],
        "local_yaw_rad": [],
        "target_x": [],
        "target_y": [],
        "target_z": [],
        "cmd_vx": [],
        "cmd_vy": [],
        "cmd_vz": [],
        "cmd_wz": [],
        "aruco_age_sec": [],
        "local_x": [],
        "local_y": [],
        "local_z": [],
    }
    modes: List[str] = []
    aruco_fresh: List[bool] = []
    local_pose_fresh: List[bool] = []

    for row in rows:
        time_sec = parse_float(row["ros_time_sec"]) - first_time
        data["time_sec"].append(time_sec)
        for key in (
            "aruco_x",
            "aruco_y",
            "aruco_z",
            "aruco_yaw_rad",
            "local_yaw_rad",
            "target_x",
            "target_y",
            "target_z",
            "cmd_vx",
            "cmd_vy",
            "cmd_vz",
            "cmd_wz",
            "aruco_age_sec",
            "local_x",
            "local_y",
            "local_z",
        ):
            data[key].append(parse_float(row[key]))
        modes.append(row["mode"].strip())
        aruco_fresh.append(parse_int(row["aruco_fresh"]) == 1)
        local_pose_fresh.append(parse_int(row["local_pose_fresh"]) == 1)

    array_data = {key: np.asarray(value, dtype=float) for key, value in data.items()}
    array_data["mode"] = np.asarray(modes, dtype=object)
    array_data["aruco_fresh"] = np.asarray(aruco_fresh, dtype=bool)
    array_data["local_pose_fresh"] = np.asarray(local_pose_fresh, dtype=bool)
    return array_data


def mask_segments(mask: np.ndarray, time_sec: np.ndarray) -> List[Tuple[float, float]]:
    """把布尔掩码转成若干时间区间，用于给 OFFBOARD 阶段加底色。"""
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


def safe_min_max(values: np.ndarray) -> Tuple[float, float]:
    """忽略 nan 后获取范围，若无有效数据则返回默认值。"""
    finite_values = values[np.isfinite(values)]
    if finite_values.size == 0:
        return -1.0, 1.0
    return float(np.min(finite_values)), float(np.max(finite_values))


def add_offboard_shading(axes: Sequence[plt.Axes], segments: Sequence[Tuple[float, float]]) -> None:
    """在所有子图上统一标记 OFFBOARD 时段。"""
    for axis in axes:
        for start_sec, end_sec in segments:
            axis.axvspan(start_sec, end_sec, color="#f4c38b", alpha=0.25, lw=0)


def save_aruco_timeseries_plot(
    run_name: str,
    output_dir: str,
    data: Dict[str, np.ndarray],
    summary: SummaryRow,
) -> None:
    """生成 ArUco 状态随时间变化图。"""
    time_sec = data["time_sec"]
    offboard_mask = data["mode"] == "OFFBOARD"
    segments = mask_segments(offboard_mask, time_sec)

    figure, axes = plt.subplots(4, 1, figsize=(16, 12), sharex=True)
    figure.suptitle("Aruco Tracking States vs Time", fontsize=15)
    add_offboard_shading(axes, segments)

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
            axis.plot(
                time_sec,
                data[target_key],
                color="#ff6b6b",
                linestyle="--",
                linewidth=1.0,
                label=target_label,
            )
        axis.set_ylabel(y_label)
        axis.grid(True, linestyle="--", alpha=0.3)
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
    figure.savefig(
        os.path.join(output_dir, f"{run_name}_plot2_aruco_timeseries.png"),
        dpi=160,
        bbox_inches="tight",
    )
    plt.close(figure)


def save_setpoint_timeseries_plot(
    run_name: str,
    output_dir: str,
    data: Dict[str, np.ndarray],
    summary: SummaryRow,
) -> None:
    """生成速度指令随时间变化图。"""
    time_sec = data["time_sec"]
    offboard_mask = data["mode"] == "OFFBOARD"
    segments = mask_segments(offboard_mask, time_sec)

    vxy_limit = summary.vxy_limit if not math.isnan(summary.vxy_limit) else 0.6
    vz_limit = summary.vz_limit if not math.isnan(summary.vz_limit) else 0.4
    wz_limit = 1.0

    figure, axes = plt.subplots(4, 1, figsize=(16, 12), sharex=True)
    figure.suptitle("Setpoint Velocity Commands vs Time", fontsize=15)
    add_offboard_shading(axes, segments)

    plot_specs = [
        ("cmd_vx", "sp_vx", "sp_vx (m/s)", vxy_limit),
        ("cmd_vy", "sp_vy", "sp_vy (m/s)", vxy_limit),
        ("cmd_vz", "sp_vz", "sp_vz (m/s)", vz_limit),
        ("cmd_wz", "sp_yaw_rate", "sp_yaw_rate (rad/s)", wz_limit),
    ]

    for axis, (data_key, label, y_label, limit_value) in zip(axes, plot_specs):
        axis.plot(time_sec, data[data_key], color="#3b8dd5", linewidth=1.5, label=label)
        axis.axhline(limit_value, color="#ff6b6b", linestyle="--", linewidth=1.0, label=f"+{label.split('_')[1]}_limit")
        axis.axhline(-limit_value, color="#ff6b6b", linestyle="--", linewidth=1.0, label=f"-{label.split('_')[1]}_limit")
        axis.set_ylabel(y_label)
        axis.grid(True, linestyle="--", alpha=0.3)
        axis.legend(loc="upper right")

    axes[-1].set_xlabel("time_sec (s)")
    figure.tight_layout(rect=(0, 0, 1, 0.97))
    figure.savefig(
        os.path.join(output_dir, f"{run_name}_plot3_setpoint_timeseries.png"),
        dpi=160,
        bbox_inches="tight",
    )
    plt.close(figure)


def save_xy_trajectory_plot(
    run_name: str,
    output_dir: str,
    data: Dict[str, np.ndarray],
    summary: SummaryRow,
) -> None:
    """生成 OFFBOARD 阶段的 ArUco XY 轨迹图。"""
    time_sec = data["time_sec"]
    offboard_mask = data["mode"] == "OFFBOARD"
    valid_mask = offboard_mask & np.isfinite(data["aruco_x"]) & np.isfinite(data["aruco_y"])

    figure, axis = plt.subplots(figsize=(9, 9))
    axis.set_title("Aruco XY Tracking Trajectory (OFFBOARD Only)")
    axis.set_xlabel("aruco_x (m)")
    axis.set_ylabel("aruco_y (m)")
    axis.grid(True, linestyle="--", alpha=0.3)
    axis.set_aspect("equal", adjustable="box")

    if np.count_nonzero(valid_mask) >= 2:
        x_values = data["aruco_x"][valid_mask]
        y_values = data["aruco_y"][valid_mask]
        t_values = time_sec[valid_mask]
        axis.plot(x_values, y_values, color="#6b7280", alpha=0.25, linewidth=1.0)
        scatter = axis.scatter(
            x_values,
            y_values,
            c=t_values,
            cmap="viridis",
            s=18,
            edgecolors="none",
            label="aruco trajectory",
        )
        axis.scatter(
            data["target_x"][valid_mask][-1],
            data["target_y"][valid_mask][-1],
            marker="x",
            color="#ef4444",
            s=90,
            linewidths=2,
            label="target",
        )
        axis.scatter(
            x_values[0],
            y_values[0],
            marker="o",
            color="#2f9e44",
            s=80,
            label="start",
        )
        axis.scatter(
            x_values[-1],
            y_values[-1],
            marker="s",
            color="#1971c2",
            s=70,
            label="end",
        )
        color_bar = figure.colorbar(scatter, ax=axis)
        color_bar.set_label("time_sec (s)")
        axis.legend(loc="upper right")
    else:
        axis.text(
            0.5,
            0.5,
            "No valid OFFBOARD XY trajectory data",
            ha="center",
            va="center",
            fontsize=14,
            transform=axis.transAxes,
        )
        axis.legend([], [], frameon=False)

    min_x, max_x = safe_min_max(data["aruco_x"][np.isfinite(data["aruco_x"])])
    min_y, max_y = safe_min_max(data["aruco_y"][np.isfinite(data["aruco_y"])])
    axis.set_xlim(min_x - 0.1, max_x + 0.1)
    axis.set_ylim(min_y - 0.1, max_y + 0.1)
    figure.tight_layout()
    figure.savefig(
        os.path.join(output_dir, f"{run_name}_plot4_aruco_xy_trajectory_offboard_only.png"),
        dpi=160,
        bbox_inches="tight",
    )
    plt.close(figure)


def save_velocity_vector_plot(
    run_name: str,
    output_dir: str,
    data: Dict[str, np.ndarray],
    sample_step: int = 10,
) -> Dict[str, float]:
    """在 marker 坐标系下绘制采样后的速度矢量，并统计其是否朝向目标。"""
    offboard_mask = data["mode"] == "OFFBOARD"
    valid_mask = (
        offboard_mask
        & np.isfinite(data["aruco_x"])
        & np.isfinite(data["aruco_y"])
        & np.isfinite(data["cmd_vx"])
        & np.isfinite(data["cmd_vy"])
        & np.isfinite(data["aruco_yaw_rad"])
        & np.isfinite(data["local_yaw_rad"])
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
        axis.text(
            0.5,
            0.5,
            "No valid OFFBOARD samples with velocity commands",
            ha="center",
            va="center",
            fontsize=14,
            transform=axis.transAxes,
        )
        figure.tight_layout()
        figure.savefig(
            os.path.join(output_dir, f"{run_name}_plot5_velocity_vectors_to_target.png"),
            dpi=160,
            bbox_inches="tight",
        )
        plt.close(figure)
        return stats

    all_x = data["aruco_x"][valid_indices]
    all_y = data["aruco_y"][valid_indices]
    sampled_x = data["aruco_x"][sampled_indices]
    sampled_y = data["aruco_y"][sampled_indices]
    sampled_vx_map = data["cmd_vx"][sampled_indices]
    sampled_vy_map = data["cmd_vy"][sampled_indices]
    sampled_tx = data["target_x"][sampled_indices]
    sampled_ty = data["target_y"][sampled_indices]
    sampled_yaw_map_marker = data["local_yaw_rad"][sampled_indices] - data["aruco_yaw_rad"][sampled_indices]

    # CSV 中速度指令是 map/ENU 坐标系，这里逆旋转回 marker 坐标系后再比较朝向。
    cos_yaw = np.cos(-sampled_yaw_map_marker)
    sin_yaw = np.sin(-sampled_yaw_map_marker)
    sampled_vx = cos_yaw * sampled_vx_map - sin_yaw * sampled_vy_map
    sampled_vy = sin_yaw * sampled_vx_map + cos_yaw * sampled_vy_map

    to_target_x = sampled_tx - sampled_x
    to_target_y = sampled_ty - sampled_y
    cmd_norm = np.hypot(sampled_vx, sampled_vy)
    target_norm = np.hypot(to_target_x, to_target_y)
    valid_alignment = (cmd_norm > 1e-6) & (target_norm > 1e-6)

    cosine_values = np.full(sampled_indices.shape, np.nan, dtype=float)
    cosine_values[valid_alignment] = (
        sampled_vx[valid_alignment] * to_target_x[valid_alignment]
        + sampled_vy[valid_alignment] * to_target_y[valid_alignment]
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

    # 绿色箭头表示速度指令与“指向 target 的向量”同向，红色表示反向。
    scale = 1.8
    if np.any(aligned_mask):
        axis.quiver(
            sampled_x[aligned_mask],
            sampled_y[aligned_mask],
            sampled_vx[aligned_mask],
            sampled_vy[aligned_mask],
            angles="xy",
            scale_units="xy",
            scale=scale,
            color="#2f9e44",
            width=0.004,
            alpha=0.9,
            label="cmd aligned to target",
        )
    if np.any(misaligned_mask):
        axis.quiver(
            sampled_x[misaligned_mask],
            sampled_y[misaligned_mask],
            sampled_vx[misaligned_mask],
            sampled_vy[misaligned_mask],
            angles="xy",
            scale_units="xy",
            scale=scale,
            color="#e03131",
            width=0.004,
            alpha=0.9,
            label="cmd away from target",
        )

    axis.scatter(
        sampled_tx[-1],
        sampled_ty[-1],
        marker="x",
        color="#ef4444",
        s=100,
        linewidths=2,
        label="target",
    )
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
    text_lines.append("cmd rotated: map -> marker")
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

    min_x, max_x = safe_min_max(data["aruco_x"][np.isfinite(data["aruco_x"])])
    min_y, max_y = safe_min_max(data["aruco_y"][np.isfinite(data["aruco_y"])])
    axis.set_xlim(min_x - 0.1, max_x + 0.1)
    axis.set_ylim(min_y - 0.1, max_y + 0.1)
    axis.legend(loc="upper right")
    figure.tight_layout()
    figure.savefig(
        os.path.join(output_dir, f"{run_name}_plot5_velocity_vectors_to_target.png"),
        dpi=160,
        bbox_inches="tight",
    )
    plt.close(figure)
    return stats


def write_run_summary(
    csv_name: str,
    output_dir: str,
    data: Dict[str, np.ndarray],
    summary: SummaryRow,
    velocity_vector_stats: Dict[str, float] | None = None,
) -> None:
    """为每个实验输出一个简短的文本摘要。"""
    time_sec = data["time_sec"]
    offboard_mask = data["mode"] == "OFFBOARD"
    offboard_duration_sec = 0.0
    if np.count_nonzero(offboard_mask) >= 2:
        offboard_duration_sec = float(time_sec[offboard_mask][-1] - time_sec[offboard_mask][0])

    target_x = data["target_x"][~np.isnan(data["target_x"])]
    target_y = data["target_y"][~np.isnan(data["target_y"])]
    target_z = data["target_z"][~np.isnan(data["target_z"])]

    lines = [
        f"csv_path: {csv_name}",
        f"status: {summary.status}",
        f"sample_count: {len(time_sec)}",
        f"duration_sec: {time_sec[-1]:.3f}",
        f"offboard_duration_sec: {offboard_duration_sec:.3f}",
        f"offboard_rows: {summary.offboard_rows}",
        f"eval_rows: {summary.eval_rows}",
        f"fresh_ratio: {summary.fresh_ratio:.3f}" if not math.isnan(summary.fresh_ratio) else "fresh_ratio: nan",
        f"max_stale_s: {summary.max_stale_s:.3f}" if not math.isnan(summary.max_stale_s) else "max_stale_s: nan",
        f"rmse_xy: {summary.rmse_xy:.3f}" if not math.isnan(summary.rmse_xy) else "rmse_xy: nan",
        f"rmse_z: {summary.rmse_z:.3f}" if not math.isnan(summary.rmse_z) else "rmse_z: nan",
        f"p95_xy: {summary.p95_xy:.3f}" if not math.isnan(summary.p95_xy) else "p95_xy: nan",
        f"target_x: {target_x[-1]:.3f}" if target_x.size else "target_x: nan",
        f"target_y: {target_y[-1]:.3f}" if target_y.size else "target_y: nan",
        f"target_z: {target_z[-1]:.3f}" if target_z.size else "target_z: nan",
        f"kp_xy: {summary.kp_xy:.3f}",
        f"kd_xy: {summary.kd_xy:.3f}",
        f"kp_z: {summary.kp_z:.3f}",
        f"vxy_limit: {summary.vxy_limit:.3f}",
        f"vz_limit: {summary.vz_limit:.3f}",
    ]
    if velocity_vector_stats is not None:
        lines.extend(
            [
                f"velocity_vector_sample_count: {int(velocity_vector_stats['sample_count'])}",
                f"velocity_vector_aligned_count: {int(velocity_vector_stats['aligned_count'])}",
                f"velocity_vector_misaligned_count: {int(velocity_vector_stats['misaligned_count'])}",
                (
                    f"velocity_vector_alignment_ratio: {velocity_vector_stats['alignment_ratio']:.3f}"
                    if not math.isnan(velocity_vector_stats["alignment_ratio"])
                    else "velocity_vector_alignment_ratio: nan"
                ),
                (
                    f"velocity_vector_mean_cosine: {velocity_vector_stats['mean_cosine']:.3f}"
                    if not math.isnan(velocity_vector_stats["mean_cosine"])
                    else "velocity_vector_mean_cosine: nan"
                ),
            ]
        )
    with open(
        os.path.join(output_dir, f"{os.path.splitext(csv_name)[0]}_plot_summary.txt"),
        "w",
        encoding="utf-8",
    ) as file:
        file.write("\n".join(lines) + "\n")


def build_overall_report(
    summary_by_name: Dict[str, SummaryRow],
    output_path: str,
) -> None:
    """输出本批实验总体分析结果，方便快速挑选有效实验。"""
    rows = sorted(summary_by_name.items(), key=lambda item: item[0])
    ok_rows = [(name, row) for name, row in rows if row.status == "ok"]
    insufficient_rows = [name for name, row in rows if row.status != "ok"]

    ranked_by_xy = sorted(
        ok_rows,
        key=lambda item: (
            math.isnan(item[1].rmse_xy),
            item[1].rmse_xy,
        ),
    )
    ranked_by_stale = sorted(
        ok_rows,
        key=lambda item: (
            math.isnan(item[1].max_stale_s),
            item[1].max_stale_s,
        ),
    )

    lines = [
        "# 4.5 xy_tracking 数据分析",
        "",
        f"- 总实验数：{len(rows)}",
        f"- 有效实验数（status=ok）：{len(ok_rows)}",
        f"- 无效实验数（insufficient_data 等）：{len(insufficient_rows)}",
        "",
        "## RMSE_XY 从优到差",
    ]

    for name, row in ranked_by_xy:
        lines.append(
            f"- {name}: rmse_xy={row.rmse_xy:.3f} m, rmse_z={row.rmse_z:.3f} m, "
            f"fresh_ratio={row.fresh_ratio:.3f}, max_stale_s={row.max_stale_s:.3f}"
        )

    lines.append("")
    lines.append("## 视觉连续性从优到差")
    for name, row in ranked_by_stale:
        lines.append(
            f"- {name}: max_stale_s={row.max_stale_s:.3f}, fresh_ratio={row.fresh_ratio:.3f}, rmse_xy={row.rmse_xy:.3f} m"
        )

    lines.append("")
    lines.append("## 无效实验")
    if insufficient_rows:
        for name in insufficient_rows:
            lines.append(f"- {name}")
    else:
        lines.append("- 无")

    lines.append("")
    lines.append("## 结论")
    if ranked_by_xy:
        best_name, best_row = ranked_by_xy[0]
        worst_name, worst_row = ranked_by_xy[-1]
        lines.append(
            f"- 最优实验为 `{best_name}`，rmse_xy={best_row.rmse_xy:.3f} m，且 fresh_ratio={best_row.fresh_ratio:.3f}。"
        )
        lines.append(
            f"- 最差有效实验为 `{worst_name}`，rmse_xy={worst_row.rmse_xy:.3f} m，"
            f"通常伴随更大的 stale 或更短的有效 OFFBOARD 段。"
        )
        zero_stale = [name for name, row in ok_rows if row.max_stale_s == 0.0]
        if zero_stale:
            lines.append(
                f"- `max_stale_s=0` 的实验共有 {len(zero_stale)} 次，说明视觉链路稳定时 XY 跟踪误差整体更小。"
            )
        stale_runs = [name for name, row in ok_rows if row.max_stale_s > 1.0]
        if stale_runs:
            lines.append(
                f"- 出现明显丢码/陈旧数据的实验共有 {len(stale_runs)} 次：{', '.join(stale_runs)}。"
            )

    with open(output_path, "w", encoding="utf-8") as file:
        file.write("\n".join(lines) + "\n")


def ensure_directory(path: str) -> None:
    """目录不存在时自动创建。"""
    os.makedirs(path, exist_ok=True)


def main() -> None:
    """程序入口：批量读取 CSV、生成图片与总结文本。"""
    parser = argparse.ArgumentParser(description="批量分析 xy_tracking CSV 并生成图片")
    parser.add_argument("--csv-dir", required=True, help="待分析 CSV 目录")
    parser.add_argument("--summary-csv", required=True, help="xy_tracking_summary.csv 路径")
    parser.add_argument("--output-dir", required=True, help="输出 plots 根目录")
    args = parser.parse_args()

    ensure_directory(args.output_dir)
    summary_by_name = load_summary(args.summary_csv, args.csv_dir)

    csv_names = sorted(
        file_name
        for file_name in os.listdir(args.csv_dir)
        if file_name.endswith(".csv")
    )
    for csv_name in csv_names:
        if csv_name not in summary_by_name:
            raise KeyError(f"summary 中缺少统计结果: {csv_name}")
        run_name = os.path.splitext(csv_name)[0]
        run_output_dir = os.path.join(args.output_dir, run_name)
        ensure_directory(run_output_dir)

        csv_path = os.path.join(args.csv_dir, csv_name)
        data = load_run_csv(csv_path)
        summary = summary_by_name[csv_name]

        save_aruco_timeseries_plot(run_name, run_output_dir, data, summary)
        save_setpoint_timeseries_plot(run_name, run_output_dir, data, summary)
        save_xy_trajectory_plot(run_name, run_output_dir, data, summary)
        velocity_vector_stats = save_velocity_vector_plot(run_name, run_output_dir, data)
        write_run_summary(csv_name, run_output_dir, data, summary, velocity_vector_stats)
        print(f"generated: {run_output_dir}")

    build_overall_report(
        summary_by_name=summary_by_name,
        output_path=os.path.join(args.output_dir, "analysis_summary.md"),
    )
    print(f"generated: {os.path.join(args.output_dir, 'analysis_summary.md')}")


if __name__ == "__main__":
    main()
