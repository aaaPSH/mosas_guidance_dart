#!/usr/bin/env python3
"""飞行数据与滤波分析脚本单元测试。"""

import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

from analyze_flight_data import (
    CSV_COLUMNS,
    AlphaFilter,
    ScalarKalmanFilter,
    apply_filters_to_flight_data,
    compute_flight_summary,
    evaluate_filtering,
    format_terminal_report,
    parse_flight_csv,
    plot_filtering_comparison,
    plot_flight_overview,
)


def create_sample_csv_content(num_frames: int = 10, launched_at_frame: int = 5) -> str:
    """生成符合 local_recorder.cpp 格式的模拟 CSV 内容。"""
    header = ",".join(CSV_COLUMNS) + "\n"
    rows = []
    base_time_ns = 1_000_000_000_000
    for i in range(num_frames):
        frame_ts = base_time_ns + i * 20_000_000  # 50 FPS, 20ms
        recv_ts = frame_ts + 2_000_000  # 2ms latency
        imu_ts = frame_ts
        found = 1 if i >= 2 else 0
        target_x = 300 + i * 2 if found else 0
        target_y = 200 + i if found else 0
        target_w = 40 if found else 0
        target_h = 40 if found else 0
        target_area = target_w * target_h
        center_x = target_x + target_w / 2.0 if found else 0.0
        center_y = target_y + target_h / 2.0 if found else 0.0

        ax = 0.5 + 0.1 * i
        ay = 0.1 * np.sin(i)
        az = 9.8 + 0.05 * i
        gx = 0.01 * i
        gy = 0.02 * i
        gz = 0.03 * i
        pitch = 0.1 + 0.01 * i
        att_valid = 1
        launched = 1 if i >= launched_at_frame else 0
        los_valid = 1 if found else 0
        los_qy = 0.05 * i if los_valid else 0.0
        los_qz = 0.02 * i if los_valid else 0.0
        los_rate_valid = 1 if (found and i > 2) else 0
        los_rate_qy = 0.1 if los_rate_valid else 0.0
        los_rate_qz = 0.05 if los_rate_valid else 0.0
        guidance_valid = 1 if (launched and los_rate_valid) else 0
        nav_ax = 1.0 if guidance_valid else 0.0
        nav_ay = 2.0 if guidance_valid else 0.0
        nav_az = 3.0 if guidance_valid else 0.0
        body_ax = 1.1 if guidance_valid else 0.0
        body_ay = 2.1 if guidance_valid else 0.0
        body_az = 3.1 if guidance_valid else 0.0
        body_nx = 0.1 if guidance_valid else 0.0
        body_ny = 0.2 if guidance_valid else 0.0
        body_nz = 0.3 if guidance_valid else 0.0
        cmd_overload = 1.5 if guidance_valid else 0.0
        cmd_phase = 0.785 if guidance_valid else 0.0

        row = [
            i, frame_ts, recv_ts, imu_ts, found,
            target_x, target_y, target_w, target_h, target_area, center_x, center_y,
            ax, ay, az, gx, gy, gz, pitch, att_valid, launched,
            los_valid, los_qy, los_qz, los_rate_valid, los_rate_qy, los_rate_qz,
            guidance_valid, nav_ax, nav_ay, nav_az, body_ax, body_ay, body_az,
            body_nx, body_ny, body_nz, cmd_overload, cmd_phase,
        ]
        rows.append(",".join(str(v) for v in row))
    return header + "\n".join(rows) + "\n"


class TestAnalyzeFlightDataParsing(unittest.TestCase):
    """测试 CSV 解析与基础飞行数据指标计算。"""

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.csv_path = Path(self.temp_dir.name) / "test_flight.csv"
        self.csv_path.write_text(create_sample_csv_content(num_frames=10, launched_at_frame=5), encoding="utf-8")

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_parse_flight_csv(self):
        df = parse_flight_csv(self.csv_path)
        self.assertEqual(len(df), 10)
        self.assertIn("time_s", df.columns)
        self.assertAlmostEqual(df["time_s"].iloc[0], 0.0)
        self.assertAlmostEqual(df["time_s"].iloc[-1], 9 * 0.02, places=4)
        self.assertEqual(df["vision_found"].sum(), 8)

    def test_compute_flight_summary(self):
        df = parse_flight_csv(self.csv_path)
        summary = compute_flight_summary(df)

        self.assertEqual(summary["total_frames"], 10)
        self.assertAlmostEqual(summary["duration_s"], 0.18, places=2)
        self.assertAlmostEqual(summary["avg_fps"], 50.0, places=1)
        self.assertAlmostEqual(summary["mean_latency_ms"], 2.0, places=2)
        self.assertTrue(summary["launched"])
        self.assertAlmostEqual(summary["launch_time_s"], 0.10, places=2)
        self.assertEqual(summary["target_found_count"], 8)
        self.assertAlmostEqual(summary["target_found_ratio"], 0.8, places=2)
        self.assertEqual(summary["max_consecutive_lost"], 2)


class TestFilteringEngine(unittest.TestCase):
    """测试离线滤波算法及其性能评估指标。"""

    def test_scalar_kalman_filter_noise_reduction(self):
        kf = ScalarKalmanFilter(process_noise=0.01, measurement_noise=1.0, initial_covariance=1.0)
        np.random.seed(42)
        true_val = 10.0
        noisy_samples = true_val + np.random.normal(0, 1.0, size=100)
        filtered = []
        for s in noisy_samples:
            filtered.append(kf.update(s, dt=0.02))

        filtered = np.array(filtered)
        raw_var = np.var(noisy_samples)
        filt_var = np.var(filtered)
        self.assertLess(filt_var, raw_var)

    def test_alpha_filter_smoothing(self):
        af = AlphaFilter(alpha=0.2)
        np.random.seed(42)
        noisy = np.random.normal(5.0, 1.0, size=100)
        filtered = [af.update(v) for v in noisy]
        self.assertLess(np.var(filtered), np.var(noisy))

    def test_evaluate_filtering_metrics(self):
        np.random.seed(42)
        t = np.linspace(0, 2, 100)
        clean = np.sin(2 * np.pi * t)
        noisy = clean + np.random.normal(0, 0.2, size=100)
        af = AlphaFilter(alpha=0.2)
        filtered = np.array([af.update(v) for v in noisy])

        metrics = evaluate_filtering(noisy, filtered, dt=0.02)
        self.assertIn("variance_reduction", metrics)
        self.assertIn("residual_mean", metrics)
        self.assertIn("residual_std", metrics)
        self.assertIn("roughness_reduction", metrics)
        self.assertGreater(metrics["roughness_reduction"], 0.0)

    def test_apply_filters_to_flight_data(self):
        temp_dir = tempfile.TemporaryDirectory()
        csv_path = Path(temp_dir.name) / "test_flight.csv"
        csv_path.write_text(create_sample_csv_content(num_frames=20, launched_at_frame=5), encoding="utf-8")
        df = parse_flight_csv(csv_path)

        df_filt, reports = apply_filters_to_flight_data(df)
        temp_dir.cleanup()

        self.assertIn("target_center_x_filt", df_filt.columns)
        self.assertIn("target_center_y_filt", df_filt.columns)
        self.assertIn("imu_accel_x_filt", df_filt.columns)
        self.assertIn("imu_accel_y_filt", df_filt.columns)
        self.assertIn("imu_accel_z_filt", df_filt.columns)
        self.assertIn("imu_gyro_x_filt", df_filt.columns)
        self.assertIn("imu_gyro_y_filt", df_filt.columns)
        self.assertIn("imu_gyro_z_filt", df_filt.columns)
        self.assertIn("los_rate_qy_filt", df_filt.columns)
        self.assertIn("los_rate_qz_filt", df_filt.columns)

        self.assertIn("target_center_x", reports)
        self.assertIn("imu_accel_x", reports)
        self.assertIn("los_rate_qy", reports)


class TestPlottingAndReporting(unittest.TestCase):
    """测试终端文本报告与图表导出功能。"""

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.csv_path = Path(self.temp_dir.name) / "test_flight.csv"
        self.csv_path.write_text(create_sample_csv_content(num_frames=20, launched_at_frame=5), encoding="utf-8")
        self.df = parse_flight_csv(self.csv_path)
        self.df_filt, self.reports = apply_filters_to_flight_data(self.df)
        self.summary = compute_flight_summary(self.df_filt)

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_format_terminal_report(self):
        report = format_terminal_report(self.summary, self.reports)
        self.assertIsInstance(report, str)
        self.assertIn("飞行数据综合分析报告", report)
        self.assertIn("滤波性能评估", report)
        self.assertIn("target_center_x", report)
        self.assertIn("imu_accel_x", report)

    def test_plot_flight_overview_generation(self):
        out_png = Path(self.temp_dir.name) / "overview.png"
        plot_flight_overview(self.df_filt, out_png, show=False)
        self.assertTrue(out_png.exists())
        self.assertGreater(out_png.stat().st_size, 1000)

    def test_plot_filtering_comparison_generation(self):
        out_png = Path(self.temp_dir.name) / "filtering.png"
        plot_filtering_comparison(self.df_filt, self.reports, out_png, show=False)
        self.assertTrue(out_png.exists())
        self.assertGreater(out_png.stat().st_size, 1000)


if __name__ == "__main__":
    unittest.main()
