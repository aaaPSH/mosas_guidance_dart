#!/usr/bin/env python3
"""飞行数据与滤波对比分析工具。

支持读取 local_recorder 记录的 CSV 飞行内录，输出基础时序统计、视觉与IMU指标、
制导指令，以及离线卡尔曼/低通滤波对比与滤波量化性能评估。
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict

import numpy as np
import pandas as pd

# 对应 local_recorder.cpp 中的 kFlightDataHeader
CSV_COLUMNS = [
    "frame_index",
    "frame_timestamp_ns",
    "frame_received_timestamp_ns",
    "imu_timestamp_ns",
    "vision_found",
    "target_x",
    "target_y",
    "target_width",
    "target_height",
    "target_area",
    "target_center_x",
    "target_center_y",
    "imu_acceleration_x_mps2",
    "imu_acceleration_y_mps2",
    "imu_acceleration_z_mps2",
    "imu_angular_velocity_x_rad_s",
    "imu_angular_velocity_y_rad_s",
    "imu_angular_velocity_z_rad_s",
    "imu_pitch_rad",
    "imu_attitude_valid",
    "imu_launched",
    "los_valid",
    "los_q_y_rad",
    "los_q_z_rad",
    "los_rate_valid",
    "los_rate_q_y_rad_s",
    "los_rate_q_z_rad_s",
    "guidance_valid",
    "body_overload_x_g",
    "body_overload_y_g",
    "body_overload_z_g",
    "command_overload_g",
    "command_phase_rad",
]


def parse_flight_csv(csv_path: Path | str) -> pd.DataFrame:
    """读取飞行数据 CSV 并添加相对时间等基础列。"""
    path = Path(csv_path)
    if not path.is_file():
        raise FileNotFoundError(f"CSV 文件不存在: {path}")

    df = pd.read_csv(path)
    if df.empty:
        raise ValueError(f"CSV 文件为空: {path}")

    # 检查并重命名或对齐列名
    missing = set(CSV_COLUMNS) - set(df.columns)
    if missing:
        raise ValueError(f"CSV 缺少必需列: {missing}")

    # 计算相对时间 (秒)
    first_ts = df["frame_timestamp_ns"].iloc[0]
    df["time_s"] = (df["frame_timestamp_ns"] - first_ts) / 1e9

    # 计算帧间隔 (毫秒) 与接收延迟 (毫秒)
    df["frame_dt_ms"] = df["frame_timestamp_ns"].diff() / 1e6
    df["latency_ms"] = (
        df["frame_received_timestamp_ns"] - df["frame_timestamp_ns"]
    ) / 1e6

    return df


def compute_flight_summary(df: pd.DataFrame) -> Dict[str, Any]:
    """计算飞行过程的各项统计指标。"""
    total_frames = len(df)
    duration_s = float(df["time_s"].iloc[-1] - df["time_s"].iloc[0]) if total_frames > 1 else 0.0
    avg_fps = float((total_frames - 1) / duration_s) if duration_s > 0 else 0.0

    # 时延与卡顿
    valid_dt = df["frame_dt_ms"].dropna()
    max_frame_interval_ms = float(valid_dt.max()) if not valid_dt.empty else 0.0
    mean_latency_ms = float(df["latency_ms"].mean()) if "latency_ms" in df.columns else 0.0

    # 起飞判定
    launched_mask = df["imu_launched"] == 1
    launched = bool(launched_mask.any())
    launch_time_s = None
    post_launch_duration_s = None
    if launched:
        launch_idx = launched_mask.idxmax()
        launch_time_s = float(df.loc[launch_idx, "time_s"])
        post_launch_duration_s = float(df["time_s"].iloc[-1] - launch_time_s)

    # 视觉跟踪
    found_series = df["vision_found"] == 1
    target_found_count = int(found_series.sum())
    target_found_ratio = float(target_found_count / total_frames) if total_frames > 0 else 0.0

    # 最大连续丢失帧数
    max_consecutive_lost = 0
    current_lost = 0
    for found in df["vision_found"]:
        if found == 0:
            current_lost += 1
            if current_lost > max_consecutive_lost:
                max_consecutive_lost = current_lost
        else:
            current_lost = 0

    # IMU 统计
    accel_norm = np.sqrt(
        df["imu_acceleration_x_mps2"] ** 2
        + df["imu_acceleration_y_mps2"] ** 2
        + df["imu_acceleration_z_mps2"] ** 2
    )
    max_accel_norm = float(accel_norm.max())

    gyro_norm = np.sqrt(
        df["imu_angular_velocity_x_rad_s"] ** 2
        + df["imu_angular_velocity_y_rad_s"] ** 2
        + df["imu_angular_velocity_z_rad_s"] ** 2
    )
    max_gyro_norm = float(gyro_norm.max())

    pitch_deg = np.degrees(df["imu_pitch_rad"])
    pitch_min_deg = float(pitch_deg.min())
    pitch_max_deg = float(pitch_deg.max())

    # 制导与过载统计
    guidance_valid_count = int((df["guidance_valid"] == 1).sum())
    max_command_overload_g = float(df["command_overload_g"].max())

    body_norm_overload = np.sqrt(
        df["body_overload_y_g"] ** 2 + df["body_overload_z_g"] ** 2
    )
    max_body_norm_overload_g = float(body_norm_overload.max())
    max_body_overload_x_g = float(df["body_overload_x_g"].abs().max())
    max_body_overload_y_g = float(df["body_overload_y_g"].abs().max())
    max_body_overload_z_g = float(df["body_overload_z_g"].abs().max())

    guidance_mask = df["guidance_valid"] == 1
    if guidance_mask.any():
        overload_err = (
            df.loc[guidance_mask, "command_overload_g"]
            - body_norm_overload[guidance_mask]
        )
        overload_rmse = float(np.sqrt(np.mean(overload_err**2)))
        overload_mean_err = float(np.mean(overload_err))
        overload_max_err = float(np.max(np.abs(overload_err)))
        cmd_phases = np.degrees(df.loc[guidance_mask, "command_phase_rad"])
        phase_mean_deg = float(cmd_phases.mean())
        phase_std_deg = float(cmd_phases.std())
    else:
        overload_rmse = 0.0
        overload_mean_err = 0.0
        overload_max_err = 0.0
        phase_mean_deg = 0.0
        phase_std_deg = 0.0

    # 分时段细化统计
    # 0.0-1.8s: 初始平稳/初导段, 1.8-2.5s: 剧烈扰动段, 2.5-6.2s: 主制导段, 6.2-10.0s: 末段/丢失段
    segments = [
        ("0.0s ~ 1.8s (初始飞行段)", 0.0, 1.8),
        ("1.8s ~ 2.5s (剧烈机动段)", 1.8, 2.5),
        ("2.5s ~ 6.2s (主导引收敛段)", 2.5, 6.2),
        ("6.2s ~ 10.0s (末段平飞/丢失段)", 6.2, 10.0),
    ]
    segment_stats = []
    for seg_name, t_start, t_end in segments:
        m = (df["time_s"] >= t_start) & (df["time_s"] < t_end)
        if m.any():
            sub_df = df[m]
            sub_acc = np.sqrt(
                sub_df["imu_acceleration_x_mps2"] ** 2
                + sub_df["imu_acceleration_y_mps2"] ** 2
                + sub_df["imu_acceleration_z_mps2"] ** 2
            ).max()
            sub_gyro = np.sqrt(
                sub_df["imu_angular_velocity_x_rad_s"] ** 2
                + sub_df["imu_angular_velocity_y_rad_s"] ** 2
                + sub_df["imu_angular_velocity_z_rad_s"] ** 2
            ).max()
            sub_cmd_g = sub_df["command_overload_g"].max()
            sub_body_g = np.sqrt(
                sub_df["body_overload_y_g"] ** 2 + sub_df["body_overload_z_g"] ** 2
            ).max()
            sub_g_valid = int((sub_df["guidance_valid"] == 1).sum())
            sub_v_found = int((sub_df["vision_found"] == 1).sum())
            segment_stats.append(
                {
                    "name": seg_name,
                    "accel_max": float(sub_acc),
                    "gyro_max": float(sub_gyro),
                    "cmd_overload_max": float(sub_cmd_g),
                    "body_overload_max": float(sub_body_g),
                    "guidance_valid_frames": sub_g_valid,
                    "vision_found_frames": sub_v_found,
                    "total_frames": len(sub_df),
                }
            )

    return {
        "total_frames": total_frames,
        "duration_s": duration_s,
        "avg_fps": avg_fps,
        "max_frame_interval_ms": max_frame_interval_ms,
        "mean_latency_ms": mean_latency_ms,
        "launched": launched,
        "launch_time_s": launch_time_s,
        "post_launch_duration_s": post_launch_duration_s,
        "target_found_count": target_found_count,
        "target_found_ratio": target_found_ratio,
        "max_consecutive_lost": max_consecutive_lost,
        "max_accel_norm": max_accel_norm,
        "max_gyro_norm": max_gyro_norm,
        "pitch_range_deg": (pitch_min_deg, pitch_max_deg),
        "guidance_valid_count": guidance_valid_count,
        "max_command_overload_g": max_command_overload_g,
        "max_body_norm_overload_g": max_body_norm_overload_g,
        "max_body_overload_x_g": max_body_overload_x_g,
        "max_body_overload_y_g": max_body_overload_y_g,
        "max_body_overload_z_g": max_body_overload_z_g,
        "overload_rmse": overload_rmse,
        "overload_mean_err": overload_mean_err,
        "overload_max_err": overload_max_err,
        "phase_mean_deg": phase_mean_deg,
        "phase_std_deg": phase_std_deg,
        "segment_stats": segment_stats,
    }


class ScalarKalmanFilter:
    """与 C++ 算法对齐的一维标量卡尔曼滤波器。"""

    def __init__(
        self,
        process_noise: float = 0.01,
        measurement_noise: float = 0.1,
        initial_covariance: float = 1.0,
    ):
        self.q = float(process_noise)
        self.r = float(measurement_noise)
        self.p = float(initial_covariance)
        self.x: float | None = None

    def initialize(self, initial_value: float) -> None:
        self.x = float(initial_value)

    def reset(self) -> None:
        self.x = None

    def update(self, measurement: float, dt: float = 0.02) -> float:
        if not np.isfinite(measurement):
            return self.x if self.x is not None else 0.0
        if self.x is None:
            self.x = float(measurement)
            return self.x

        dt_eff = max(float(dt), 1e-4)
        p_pred = self.p + self.q * dt_eff
        k = p_pred / (p_pred + self.r)
        self.x = self.x + k * (float(measurement) - self.x)
        self.p = (1.0 - k) * p_pred
        return self.x


class AlphaFilter:
    """与 C++ attitude_estimator 对齐的指数移动平均/低通滤波器。"""

    def __init__(self, alpha: float = 0.2):
        self.alpha = float(alpha)
        self.y: float | None = None

    def initialize(self, val: float) -> None:
        self.y = float(val)

    def reset(self) -> None:
        self.y = None

    def update(self, measurement: float) -> float:
        if not np.isfinite(measurement):
            return self.y if self.y is not None else 0.0
        if self.y is None:
            self.y = float(measurement)
            return self.y
        self.y = self.alpha * float(measurement) + (1.0 - self.alpha) * self.y
        return self.y


def evaluate_filtering(
    raw: np.ndarray, filtered: np.ndarray, dt: float = 0.02
) -> Dict[str, float]:
    """计算滤波性能评估指标：方差缩减比、残差均值/标准差、时延、粗糙度变化。"""
    raw = np.asarray(raw, dtype=float)
    filtered = np.asarray(filtered, dtype=float)

    mask = np.isfinite(raw) & np.isfinite(filtered)
    if not np.any(mask):
        return {
            "variance_reduction": 0.0,
            "residual_mean": 0.0,
            "residual_std": 0.0,
            "roughness_reduction": 0.0,
            "estimated_lag_ms": 0.0,
        }

    r_valid = raw[mask]
    f_valid = filtered[mask]

    # 方差缩减
    raw_var = float(np.var(r_valid))
    filt_var = float(np.var(f_valid))
    variance_reduction = float(1.0 - filt_var / raw_var) if raw_var > 1e-12 else 0.0

    # 残差分析
    residuals = r_valid - f_valid
    residual_mean = float(np.mean(residuals))
    residual_std = float(np.std(residuals))

    # 全变差（粗糙度）缩减比
    if len(r_valid) > 1:
        tv_raw = float(np.sum(np.abs(np.diff(r_valid))))
        tv_filt = float(np.sum(np.abs(np.diff(f_valid))))
        roughness_reduction = (
            float(1.0 - tv_filt / tv_raw) if tv_raw > 1e-12 else 0.0
        )
    else:
        roughness_reduction = 0.0

    # 基于互相关的时延估计 (ms)
    estimated_lag_ms = 0.0
    if len(r_valid) > 5 and raw_var > 1e-12 and filt_var > 1e-12:
        r_centered = r_valid - np.mean(r_valid)
        f_centered = f_valid - np.mean(f_valid)
        corr = np.correlate(r_centered, f_centered, mode="full")
        lags = np.arange(-len(r_valid) + 1, len(r_valid))
        peak_lag = lags[np.argmax(corr)]
        estimated_lag_ms = float(peak_lag * dt * 1000.0)

    return {
        "variance_reduction": variance_reduction,
        "residual_mean": residual_mean,
        "residual_std": residual_std,
        "roughness_reduction": roughness_reduction,
        "estimated_lag_ms": estimated_lag_ms,
    }


def apply_filters_to_flight_data(
    df: pd.DataFrame,
) -> tuple[pd.DataFrame, Dict[str, Dict[str, float]]]:
    """对飞行数据应用离线滤波并生成滤波分析报告。"""
    df_out = df.copy()
    dt = float(df_out["time_s"].diff().median()) if len(df_out) > 1 else 0.02
    if not np.isfinite(dt) or dt <= 0:
        dt = 0.02

    reports: Dict[str, Dict[str, float]] = {}

    # 1. 目标中心位置卡尔曼滤波 (仅在 vision_found == 1 时跟踪更新)
    kf_cx = ScalarKalmanFilter(process_noise=0.05, measurement_noise=1.0)
    kf_cy = ScalarKalmanFilter(process_noise=0.05, measurement_noise=1.0)
    cx_filt = []
    cy_filt = []
    for found, cx, cy in zip(
        df_out["vision_found"], df_out["target_center_x"], df_out["target_center_y"]
    ):
        if found == 1:
            cx_filt.append(kf_cx.update(cx, dt))
            cy_filt.append(kf_cy.update(cy, dt))
        else:
            kf_cx.reset()
            kf_cy.reset()
            cx_filt.append(np.nan)
            cy_filt.append(np.nan)

    df_out["target_center_x_filt"] = cx_filt
    df_out["target_center_y_filt"] = cy_filt

    # 仅对 found 帧评估视觉滤波指标
    found_mask = df_out["vision_found"] == 1
    if found_mask.sum() > 2:
        reports["target_center_x"] = evaluate_filtering(
            df_out.loc[found_mask, "target_center_x"].to_numpy(),
            df_out.loc[found_mask, "target_center_x_filt"].to_numpy(),
            dt,
        )
        reports["target_center_y"] = evaluate_filtering(
            df_out.loc[found_mask, "target_center_y"].to_numpy(),
            df_out.loc[found_mask, "target_center_y_filt"].to_numpy(),
            dt,
        )

    # 2. IMU 加速度与角速度低通滤波
    imu_channels = [
        ("imu_accel_x", "imu_acceleration_x_mps2", "imu_accel_x_filt"),
        ("imu_accel_y", "imu_acceleration_y_mps2", "imu_accel_y_filt"),
        ("imu_accel_z", "imu_acceleration_z_mps2", "imu_accel_z_filt"),
        ("imu_gyro_x", "imu_angular_velocity_x_rad_s", "imu_gyro_x_filt"),
        ("imu_gyro_y", "imu_angular_velocity_y_rad_s", "imu_gyro_y_filt"),
        ("imu_gyro_z", "imu_angular_velocity_z_rad_s", "imu_gyro_z_filt"),
    ]
    for rep_name, raw_col, filt_col in imu_channels:
        af = AlphaFilter(alpha=0.2)
        filt_series = [af.update(v) for v in df_out[raw_col]]
        df_out[filt_col] = filt_series
        reports[rep_name] = evaluate_filtering(
            df_out[raw_col].to_numpy(), np.array(filt_series), dt
        )

    # 3. 视线角速率滤波
    kf_qy = ScalarKalmanFilter(process_noise=0.01, measurement_noise=0.1)
    kf_qz = ScalarKalmanFilter(process_noise=0.01, measurement_noise=0.1)
    df_out["los_rate_qy_filt"] = [
        kf_qy.update(v, dt) if valid == 1 else 0.0
        for valid, v in zip(df_out["los_rate_valid"], df_out["los_rate_q_y_rad_s"])
    ]
    df_out["los_rate_qz_filt"] = [
        kf_qz.update(v, dt) if valid == 1 else 0.0
        for valid, v in zip(df_out["los_rate_valid"], df_out["los_rate_q_z_rad_s"])
    ]

    los_mask = df_out["los_rate_valid"] == 1
    if los_mask.sum() > 2:
        reports["los_rate_qy"] = evaluate_filtering(
            df_out.loc[los_mask, "los_rate_q_y_rad_s"].to_numpy(),
            df_out.loc[los_mask, "los_rate_qy_filt"].to_numpy(),
            dt,
        )
        reports["los_rate_qz"] = evaluate_filtering(
            df_out.loc[los_mask, "los_rate_q_z_rad_s"].to_numpy(),
            df_out.loc[los_mask, "los_rate_qz_filt"].to_numpy(),
            dt,
        )

    return df_out, reports


def format_terminal_report(
    summary: Dict[str, Any], filter_reports: Dict[str, Dict[str, float]]
) -> str:
    """生成格式化的终端分析报告文本。"""
    lines = []
    lines.append("=" * 72)
    lines.append("                     飞行数据综合分析报告")
    lines.append("=" * 72)
    lines.append(f"总帧数:           {summary['total_frames']} 帧")
    lines.append(f"总时长:           {summary['duration_s']:.2f} 秒")
    lines.append(f"平均帧率:         {summary['avg_fps']:.1f} FPS")
    lines.append(f"最大帧间隔:       {summary['max_frame_interval_ms']:.2f} ms")
    lines.append(f"平均接收延迟:     {summary['mean_latency_ms']:.2f} ms")
    lines.append("-" * 72)
    if summary["launched"]:
        lines.append(
            f"起飞状态:         已起飞 (时刻: {summary['launch_time_s']:.2f}s, 飞行时长: {summary['post_launch_duration_s']:.2f}s)"
        )
    else:
        lines.append("起飞状态:         未触发起飞")
    lines.append(
        f"视觉目标捕获率:   {summary['target_found_ratio']*100:.1f}% ({summary['target_found_count']}/{summary['total_frames']} 帧)"
    )
    lines.append(f"最大连续丢失帧数: {summary['max_consecutive_lost']} 帧")
    lines.append(f"IMU 最大加速度:   {summary['max_accel_norm']:.2f} m/s^2")
    lines.append(f"IMU 最大角速度:   {summary['max_gyro_norm']:.2f} rad/s")
    lines.append(
        f"俯仰角范围:       [{summary['pitch_range_deg'][0]:.1f}°, {summary['pitch_range_deg'][1]:.1f}°]"
    )
    lines.append(f"制导有效帧数:     {summary['guidance_valid_count']} 帧")
    lines.append(f"最大指令过载:     {summary['max_command_overload_g']:.2f} g")
    lines.append("=" * 72)
    lines.append("                 滤波性能评估 (Filtering Analysis)")
    lines.append("=" * 72)
    lines.append(
        f"{'通道名称':<18} | {'降噪比':<8} | {'全变差缩减':<10} | {'残差均值':<10} | {'残差标准差':<10} | {'估计时延':<10}"
    )
    lines.append("-" * 72)
    for channel, m in filter_reports.items():
        var_red = f"{m['variance_reduction']*100:.1f}%"
        tv_red = f"{m['roughness_reduction']*100:.1f}%"
        r_mean = f"{m['residual_mean']:.4f}"
        r_std = f"{m['residual_std']:.4f}"
        lag = f"{m['estimated_lag_ms']:.1f}ms"
        lines.append(
            f"{channel:<18} | {var_red:<8} | {tv_red:<10} | {r_mean:<10} | {r_std:<10} | {lag:<10}"
        )
    lines.append("=" * 72)
    lines.append("           IMU 与过载指令精细分析 (Detailed IMU & Overload)")
    lines.append("=" * 72)
    lines.append(f"最大指令过载 (Command):   {summary.get('max_command_overload_g', 0.0):.2f} g")
    lines.append(f"最大机体法向过载 (|nyz|): {summary.get('max_body_norm_overload_g', 0.0):.2f} g")
    lines.append(
        f"机体各轴最大过载:         X: {summary.get('max_body_overload_x_g', 0.0):.2f}g, "
        f"Y: {summary.get('max_body_overload_y_g', 0.0):.2f}g, "
        f"Z: {summary.get('max_body_overload_z_g', 0.0):.2f}g"
    )
    lines.append(
        f"过载指令跟踪误差 (RMSE):  {summary.get('overload_rmse', 0.0):.4f} g "
        f"(均值: {summary.get('overload_mean_err', 0.0):.4f}g, 最大: {summary.get('overload_max_err', 0.0):.4f}g)"
    )
    lines.append(
        f"指令相位分布 (Phase):     均值 {summary.get('phase_mean_deg', 0.0):.1f}°, "
        f"标准差 {summary.get('phase_std_deg', 0.0):.1f}°"
    )
    lines.append("-" * 72)
    lines.append("时段细分统计:")
    lines.append(
        f"{'时段划分':<24} | {'IMU加速度':<10} | {'IMU角速度':<10} | {'机体过载':<8} | {'指令过载':<8} | {'制导/总帧'}"
    )
    lines.append("-" * 72)
    for seg in summary.get("segment_stats", []):
        name = seg["name"]
        acc_str = f"{seg['accel_max']:.1f} m/s²"
        gyro_str = f"{seg['gyro_max']:.2f} rad/s"
        body_g_str = f"{seg['body_overload_max']:.2f} g"
        cmd_g_str = f"{seg['cmd_overload_max']:.2f} g"
        frame_str = f"{seg['guidance_valid_frames']}/{seg['total_frames']}"
        lines.append(
            f"{name:<24} | {acc_str:<10} | {gyro_str:<10} | {body_g_str:<8} | {cmd_g_str:<8} | {frame_str}"
        )
    lines.append("=" * 72)
    return "\n".join(lines)


def plot_flight_overview(
    df: pd.DataFrame, output_path: Path | str, show: bool = False
) -> None:
    """绘制飞行综合概览图并保存。"""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    t = df["time_s"]

    # 1. 视觉目标面板
    ax1 = axes[0]
    ax1.plot(t, df["target_center_x"], label="Target X (px)", color="royalblue", alpha=0.8)
    ax1.plot(t, df["target_center_y"], label="Target Y (px)", color="crimson", alpha=0.8)
    ax1.set_ylabel("Pixel Coord")
    ax1.set_title("1. Vision Tracking (Target Center)")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right")
    lost_mask = df["vision_found"] == 0
    if lost_mask.any():
        ax1.fill_between(
            t,
            ax1.get_ylim()[0],
            ax1.get_ylim()[1],
            where=lost_mask,
            color="gray",
            alpha=0.2,
            label="Lost",
        )

    # 2. IMU 动力学面板
    ax2 = axes[1]
    ax2.plot(t, df["imu_acceleration_x_mps2"], label="Accel X", color="red", alpha=0.7)
    ax2.plot(t, df["imu_acceleration_y_mps2"], label="Accel Y", color="green", alpha=0.7)
    ax2.plot(t, df["imu_acceleration_z_mps2"], label="Accel Z", color="blue", alpha=0.7)
    ax2.set_ylabel("Accel (m/s²)")
    ax2.set_title("2. IMU Dynamics (Acceleration)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    launched_mask = df["imu_launched"] == 1
    if launched_mask.any():
        launch_t = t.iloc[launched_mask.idxmax()]
        ax2.axvline(
            launch_t,
            color="purple",
            linestyle="--",
            linewidth=2,
            label=f"Launch ({launch_t:.2f}s)",
        )
    overload_active = (df["command_overload_g"] > 0.05) | (df["guidance_valid"] == 1)
    if overload_active.any():
        y2_min, y2_max = ax2.get_ylim()
        ax2.fill_between(
            t,
            y2_min,
            y2_max,
            where=overload_active,
            color="lightcoral",
            alpha=0.2,
            label="Overload Active",
            zorder=0,
        )
        ax2.set_ylim(y2_min, y2_max)
    ax2.legend(loc="upper right")

    # 3. 视线角与角速率面板
    ax3 = axes[2]
    ax3.plot(t, df["los_q_y_rad"], label="LOS q_y (rad)", color="darkorange")
    ax3.plot(t, df["los_q_z_rad"], label="LOS q_z (rad)", color="teal")
    ax3.plot(
        t,
        df["los_rate_q_y_rad_s"],
        label="LOS Rate q_y (rad/s)",
        color="coral",
        linestyle=":",
    )
    ax3.plot(
        t,
        df["los_rate_q_z_rad_s"],
        label="LOS Rate q_z (rad/s)",
        color="mediumturquoise",
        linestyle=":",
    )
    ax3.set_ylabel("Rad / Rad/s")
    ax3.set_title("3. Line of Sight (LOS) Angles & Angular Rates")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right")

    # 4. 制导过载与指令面板
    ax4 = axes[3]
    ax4.plot(t, df["body_overload_y_g"], label="Body Overload Y (g)", color="mediumpurple")
    ax4.plot(t, df["body_overload_z_g"], label="Body Overload Z (g)", color="darkviolet")
    ax4.plot(
        t,
        df["command_overload_g"],
        label="Command Overload (g)",
        color="firebrick",
        linewidth=2,
    )
    ax4.set_ylabel("Overload (g)")
    ax4.set_xlabel("Time (s)")
    ax4.set_title("4. Guidance & Overload Commands")
    ax4.grid(True, linestyle="--", alpha=0.5)
    ax4.legend(loc="upper right")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def plot_filtering_comparison(
    df: pd.DataFrame,
    reports: Dict[str, Dict[str, float]],
    output_path: Path | str,
    show: bool = False,
) -> None:
    """绘制滤波前后对比与滤波分析图并保存。"""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    t = df["time_s"]

    # 1. 目标位置滤波对比
    ax1 = axes[0]
    ax1.plot(t, df["target_center_x"], ".", label="Raw Center X", color="lightskyblue", alpha=0.7)
    ax1.plot(t, df["target_center_x_filt"], "-", label="Kalman Filtered X", color="blue", linewidth=1.5)
    ax1.plot(t, df["target_center_y"], ".", label="Raw Center Y", color="lightpink", alpha=0.7)
    ax1.plot(t, df["target_center_y_filt"], "-", label="Kalman Filtered Y", color="crimson", linewidth=1.5)
    ax1.set_ylabel("Center (px)")
    ax1.set_title("1. Vision Target Filtering (Kalman Filter)")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right")

    # 2. IMU 加速度滤波对比
    ax2 = axes[1]
    ax2.plot(t, df["imu_acceleration_x_mps2"], label="Raw Accel X", color="lightcoral", alpha=0.5)
    ax2.plot(t, df["imu_accel_x_filt"], label="Alpha Filtered Accel X", color="darkred", linewidth=1.5)
    ax2.plot(t, df["imu_acceleration_z_mps2"], label="Raw Accel Z", color="cornflowerblue", alpha=0.5)
    ax2.plot(t, df["imu_accel_z_filt"], label="Alpha Filtered Accel Z", color="navy", linewidth=1.5)
    ax2.set_ylabel("Accel (m/s²)")
    ax2.set_title("2. IMU Acceleration Low-Pass Filtering (Alpha Filter)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper right")

    # 3. IMU 角速度滤波对比
    ax3 = axes[2]
    ax3.plot(t, df["imu_angular_velocity_x_rad_s"], label="Raw Gyro X", color="lightgreen", alpha=0.5)
    ax3.plot(t, df["imu_gyro_x_filt"], label="Filtered Gyro X", color="forestgreen", linewidth=1.5)
    ax3.plot(t, df["imu_angular_velocity_z_rad_s"], label="Raw Gyro Z", color="violet", alpha=0.5)
    ax3.plot(t, df["imu_gyro_z_filt"], label="Filtered Gyro Z", color="purple", linewidth=1.5)
    ax3.set_ylabel("Angular Vel (rad/s)")
    ax3.set_title("3. IMU Angular Velocity Filtering")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right")

    # 4. 视线角速率滤波对比
    ax4 = axes[3]
    ax4.plot(t, df["los_rate_q_y_rad_s"], label="Raw LOS Rate q_y", color="sandybrown", alpha=0.6)
    ax4.plot(t, df["los_rate_qy_filt"], label="Filtered LOS Rate q_y", color="saddlebrown", linewidth=1.5)
    ax4.plot(t, df["los_rate_q_z_rad_s"], label="Raw LOS Rate q_z", color="skyblue", alpha=0.6)
    ax4.plot(t, df["los_rate_qz_filt"], label="Filtered LOS Rate q_z", color="darkcyan", linewidth=1.5)
    ax4.set_ylabel("LOS Rate (rad/s)")
    ax4.set_xlabel("Time (s)")
    ax4.set_title("4. Line of Sight Rate Filtering (Kalman Filter)")
    ax4.grid(True, linestyle="--", alpha=0.5)
    ax4.legend(loc="upper right")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def plot_imu_and_overload_detailed(
    df: pd.DataFrame, output_path: Path | str, show: bool = False
) -> None:
    """绘制 IMU 动力学与过载控制指令高精度细分度图表并保存。"""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import AutoMinorLocator, MultipleLocator

    fig, axes = plt.subplots(4, 1, figsize=(16, 14), sharex=True)
    t = df["time_s"]

    # 1. IMU 三轴加速度 (细分度)
    ax1 = axes[0]
    ax1.plot(t, df["imu_acceleration_x_mps2"], label="Accel X (m/s²)", color="crimson", linewidth=1.2, alpha=0.85)
    ax1.plot(t, df["imu_acceleration_y_mps2"], label="Accel Y (m/s²)", color="forestgreen", linewidth=1.2, alpha=0.85)
    ax1.plot(t, df["imu_acceleration_z_mps2"], label="Accel Z (m/s²)", color="royalblue", linewidth=1.2, alpha=0.85)
    accel_norm = np.sqrt(
        df["imu_acceleration_x_mps2"] ** 2
        + df["imu_acceleration_y_mps2"] ** 2
        + df["imu_acceleration_z_mps2"] ** 2
    )
    ax1.plot(t, accel_norm, label="Total Accel Norm (m/s²)", color="black", linewidth=1.2, linestyle="--", alpha=0.7)
    ax1.set_ylabel("Accel (m/s²)", fontsize=11)
    ax1.set_title("1. IMU 3-Axis Acceleration & Total Norm (High-Precision Grid)", fontsize=12, fontweight="bold")

    overload_active = (df["command_overload_g"] > 0.05) | (df["guidance_valid"] == 1)
    if overload_active.any():
        y1_min, y1_max = ax1.get_ylim()
        ax1.fill_between(
            t,
            y1_min,
            y1_max,
            where=overload_active,
            color="lightcoral",
            alpha=0.18,
            label="Overload Active",
            zorder=0,
        )
        ax1.set_ylim(y1_min, y1_max)
    ax1.legend(loc="upper right", ncol=5, framealpha=0.9)

    # 2. IMU 三轴角速度 (细分度)
    ax2 = axes[1]
    ax2.plot(t, df["imu_angular_velocity_x_rad_s"], label="Gyro X (Roll, rad/s)", color="purple", linewidth=1.2, alpha=0.85)
    ax2.plot(t, df["imu_angular_velocity_y_rad_s"], label="Gyro Y (Pitch, rad/s)", color="darkorange", linewidth=1.2, alpha=0.85)
    ax2.plot(t, df["imu_angular_velocity_z_rad_s"], label="Gyro Z (Yaw, rad/s)", color="teal", linewidth=1.2, alpha=0.85)
    ax2.set_ylabel("Angular Vel (rad/s)", fontsize=11)
    ax2.set_title("2. IMU 3-Axis Angular Velocity (High-Precision Grid)", fontsize=12, fontweight="bold")

    if overload_active.any():
        y2_min, y2_max = ax2.get_ylim()
        ax2.fill_between(
            t,
            y2_min,
            y2_max,
            where=overload_active,
            color="lightcoral",
            alpha=0.18,
            label="Overload Active",
            zorder=0,
        )
        ax2.set_ylim(y2_min, y2_max)
    ax2.legend(loc="upper right", ncol=4, framealpha=0.9)

    # 3. 机体过载与指令过载对比 (细分度)
    ax3 = axes[2]
    body_overload_norm = np.sqrt(
        df["body_overload_y_g"] ** 2 + df["body_overload_z_g"] ** 2
    )
    ax3.plot(t, df["body_overload_y_g"], label="Body Overload Y (g)", color="mediumseagreen", linewidth=1.2, alpha=0.8)
    ax3.plot(t, df["body_overload_z_g"], label="Body Overload Z (g)", color="mediumpurple", linewidth=1.2, alpha=0.8)
    ax3.plot(t, body_overload_norm, label="Body Normal Overload |nyz| (g)", color="navy", linewidth=1.6, linestyle="-")
    ax3.plot(t, df["command_overload_g"], label="Command Overload (g)", color="red", linewidth=2.0, linestyle="--")
    ax3.set_ylabel("Overload (g)", fontsize=11)
    ax3.set_title("3. Body Overload Response vs. Guidance Command Overload", fontsize=12, fontweight="bold")
    ax3.legend(loc="upper right", ncol=4, framealpha=0.9)

    # 4. 指令相位与制导有效性
    ax4 = axes[3]
    phase_deg = np.degrees(df["command_phase_rad"])
    ax4.plot(t, phase_deg, label="Command Phase (°)", color="chocolate", linewidth=1.3)
    ax4.set_ylabel("Phase Angle (°)", fontsize=11)
    ax4.set_xlabel("Time (s)", fontsize=11)
    ax4.set_title("4. Guidance Command Phase Angle & Valid Intervals", fontsize=12, fontweight="bold")

    guidance_valid_mask = df["guidance_valid"] == 1
    if guidance_valid_mask.any():
        ax4.fill_between(
            t, -180, 180, where=guidance_valid_mask, color="limegreen", alpha=0.18, label="Guidance Active"
        )
    ax4.set_ylim(-185, 185)
    ax4.yaxis.set_major_locator(MultipleLocator(45))
    ax4.yaxis.set_minor_locator(MultipleLocator(15))
    ax4.legend(loc="upper right", ncol=2, framealpha=0.9)

    # 精细化刻度与主次网格设置 (所有子图)
    for i, ax in enumerate(axes):
        ax.xaxis.set_major_locator(MultipleLocator(0.5))    # 主刻度 0.5s
        ax.xaxis.set_minor_locator(MultipleLocator(0.1))    # 次刻度 0.1s
        if i < 3:
            ax.yaxis.set_minor_locator(AutoMinorLocator(2)) # Y 轴次刻度细分
        ax.grid(True, which="major", color="#b0b0b0", linestyle="-", linewidth=0.7, alpha=0.75)
        ax.grid(True, which="minor", color="#e0e0e0", linestyle=":", linewidth=0.5, alpha=0.7)
        ax.minorticks_on()
        ax.set_xlim(t.iloc[0], t.iloc[-1])

    plt.tight_layout()
    plt.savefig(output_path, dpi=200)
    if show:
        plt.show()
    plt.close(fig)


def main() -> None:
    """CLI 入口点。"""
    parser = argparse.ArgumentParser(description="飞行数据与滤波对比分析工具")
    parser.add_argument("csv_path", type=str, help="输入飞行数据 CSV 文件路径")
    parser.add_argument("--output-dir", type=str, default=None, help="分析结果与图表输出目录")
    parser.add_argument("--no-plots", action="store_true", help="不生成图表，仅在终端打印分析报告")
    parser.add_argument("--show", action="store_true", help="交互式弹窗展示图表")

    args = parser.parse_args()
    csv_file = Path(args.csv_path)

    df = parse_flight_csv(csv_file)
    df_filt, reports = apply_filters_to_flight_data(df)
    summary = compute_flight_summary(df_filt)

    report_text = format_terminal_report(summary, reports)
    print(report_text)

    if not args.no_plots:
        output_dir = (
            Path(args.output_dir)
            if args.output_dir
            else csv_file.parent / f"{csv_file.stem}_analysis"
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        overview_path = output_dir / "flight_analysis_overview.png"
        filtering_path = output_dir / "flight_filtering_comparison.png"
        detailed_imu_path = output_dir / "flight_imu_overload_detailed.png"

        plot_flight_overview(df_filt, overview_path, show=args.show)
        plot_filtering_comparison(df_filt, reports, filtering_path, show=args.show)
        plot_imu_and_overload_detailed(df_filt, detailed_imu_path, show=args.show)
        print(f"\n图表已保存至: {output_dir}")
        print(f"  - 飞行综合概览: {overview_path}")
        print(f"  - 滤波对比分析: {filtering_path}")
        print(f"  - IMU与过载精细图: {detailed_imu_path}")


if __name__ == "__main__":
    main()
