#!/usr/bin/env python3
"""验证原程序负载下的数据记录及相对时延标定，无需真实硬件。"""

from contextlib import redirect_stderr
import csv
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/calibrate_camera_imu_sync.py"
sys.path.insert(0, str(SCRIPT.parent))
import calibrate_camera_imu_sync as calibration

cv2.setNumThreads(1)


def motion(times):
    """生成起始静止、在观测窗口内角速率非周期变化的解析旋转。"""
    u = np.maximum(times - 2.5, 0.0)
    frequencies = np.array([1.7, 4.1, 7.3])
    amplitudes = np.array([0.14, 0.10, 0.04])
    phase = u[:, None] * frequencies
    base = np.sin(phase) @ amplitudes
    derivative = np.cos(phase) @ (amplitudes * frequencies)
    envelope = 1.0 - np.exp(-u * u)
    return envelope * base, envelope * derivative + 2 * u * np.exp(-u * u) * base


def dataset(delay_ms, fps=60):
    """使用不同刚性安装轴，独立生成视觉姿态及带零偏的陀螺仪数据。"""
    imu_seconds = np.arange(0, 18, 0.005)
    camera_seconds = np.arange(0.2, 17.7, 1.0 / fps)
    camera_seconds = camera_seconds[np.arange(len(camera_seconds)) % 17 != 0]
    angles, _ = motion(camera_seconds)
    _, speed = motion(imu_seconds)
    axis = np.array([0.3, -0.4, 0.5])
    axis /= np.linalg.norm(axis)
    rotations = np.array([cv2.Rodrigues(axis * angle)[0] for angle in angles])
    gyro = np.column_stack((np.zeros_like(speed), speed, np.zeros_like(speed)))
    gyro += np.array([0.003, -0.004, 0.002])
    origin = 912_345_678_900_000
    camera_ns = origin + np.rint((camera_seconds + delay_ms / 1000) * 1e9).astype(np.int64)
    imu_ns = origin + np.rint(imu_seconds * 1e9).astype(np.int64)
    return camera_ns, rotations, imu_ns, gyro


FRAME_FIELDS = ["frame_index", "runtime_timestamp_ns", "timestamp_ns", "received_timestamp_ns",
                "observed_timestamp_ns", "timestamp_source", "offset", "length", "width", "height"]
IMU_FIELDS = ["sample_index", "runtime_timestamp_ns", "timestamp_ns", "observed_timestamp_ns",
              "acceleration_x_mps2", "acceleration_y_mps2", "acceleration_z_mps2",
              "angular_velocity_x_rad_s", "angular_velocity_y_rad_s", "angular_velocity_z_rad_s",
              "servo_control_p_normalized", "servo_control_y_normalized"]


def write_csv(path, fields, rows):
    """生成确定性的 CSV 字节，使用与 C++ 记录器一致的 LF 换行。"""
    with path.open("w", newline="", encoding="ascii") as output:
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(fields)
        writer.writerows(rows)


def render_board(rotation, matrix, distortion):
    """在已经修正 180 度倒装的正立像素方向中渲染带畸变的棋盘格。"""
    image = np.full((480, 640), 210, np.uint8)
    high_matrix = matrix.copy()
    high_matrix[:2] *= 2
    # 倾斜棋盘格，避免正对平面时的 PnP 姿态歧义干扰时延测试。
    oblique = cv2.Rodrigues(np.array([0.35, -0.25, 0.05]))[0]
    vector = cv2.Rodrigues(oblique @ rotation)[0]
    for y in range(-1, 6):
        for x in range(-1, 9):
            square = np.array([[x, y, 0], [x + 1, y, 0],
                               [x + 1, y + 1, 0], [x, y + 1, 0]], np.float64) * 0.03
            pixels, _ = cv2.projectPoints(square, vector, np.array([-0.12, -0.075, 0.85]),
                                         high_matrix, distortion)
            cv2.fillConvexPoly(image, np.rint(pixels * 256).astype(np.int32),
                              245 if (x + y) % 2 else 15, lineType=cv2.LINE_AA, shift=8)
    return cv2.resize(image, (320, 240), interpolation=cv2.INTER_AREA)


def capture_fixture(root, rendered=False):
    """写入带非零时间补偿和正立图像内参的 version-2 采集数据。"""
    camera_ns, rotations, imu_ns, gyro = dataset(8, fps=30)
    if not rendered:
        camera_ns, rotations = camera_ns[:2], rotations[:2]
        imu_ns, gyro = imu_ns[:2], gyro[:2]
    matrix = np.array([[450., 0, 149.3], [0, 440., 110.7], [0, 0, 1]])
    distortion = np.array([0.025, -0.01, 0.001, -0.0007, 0.002])
    intrinsics = {"image_orientation": "runtime_prepared_180", "image_width": 320, "image_height": 240,
                  "camera_matrix": matrix.tolist(), "distortion_coefficients": distortion.tolist()}
    (root / "intrinsics.json").write_text(json.dumps(intrinsics))
    rows = []
    with (root / "frames.gray8").open("wb") as output:
        for index, (timestamp, rotation) in enumerate(zip(camera_ns, rotations)):
            prepared = render_board(rotation, matrix, distortion) if rendered else np.zeros((240, 320), np.uint8)
            raw = cv2.flip(prepared, -1)
            output.write(raw.tobytes())
            rows.append([index, int(timestamp) - 2_000_000, int(timestamp), int(timestamp) + 4_000_000,
                         int(timestamp) + 5_000_000, "sdk_monotonic", index * raw.size, raw.size, 320, 240])
    write_csv(root / "frames.csv", FRAME_FIELDS, rows)
    write_csv(root / "imu.csv", IMU_FIELDS,
              [[index, int(timestamp) - 3_000_000, int(timestamp), int(timestamp) + 6_000_000,
                0, 9.8, 0, *sample, 0, 0] for index, (timestamp, sample) in enumerate(zip(imu_ns, gyro))])
    metadata = {
        "version": 2, "complete": True, "end_reason": "duration_elapsed", "recording_error": "none",
        "image_orientation": "sdk_raw", "analysis_image_orientation": "runtime_prepared_180",
        "physical_mount_rotation_deg": 180, "prepare_rotation_deg": 180,
        "raw_width": 320, "raw_height": 240, "prepared_width": 320, "prepared_height": 240,
        "camera_timestamp_bias_ns": -2_000_000, "measurement_latency_ns": 3_000_000,
        "camera_clock_domain": "monotonic", "imu_timestamp_source": "sequence_fixed_period",
        "imu_timestamp_quality": "estimated", "control_output_disabled": True,
        "recording": {"camera_seen": len(rows), "camera_selected": len(rows), "camera_written": len(rows),
                      "camera_dropped": 0, "imu_accepted": len(imu_ns), "imu_written": len(imu_ns),
                      "imu_dropped": 0, "pixel_bytes": len(rows) * 320 * 240,
                      "data_bytes": sum((root / name).stat().st_size
                                        for name in ("frames.gray8", "frames.csv", "imu.csv"))},
        "imu_source": {"available": True, **{name: 0 for name in (
            "parse_error_count", "dropped_chunk_count", "sequence_gap_count", "crc_error_count",
            "dropped_sample_count")}}, "camera": {"queue_dropped": 0, "timestamp_dropped": 0},
        "runtime_timing": {"vision": {"samples": len(rows)}, "guidance": {"samples": 0}},
    }
    (root / "metadata.json").write_text(json.dumps(metadata))
    return metadata


class CalibrationTest(unittest.TestCase):
    """验证偏移符号、可观测性、稳定性诊断及输入完整性。"""

    def test_positive_and_negative_offsets(self):
        for delay_ms in (7.4, -5.7):
            with self.subTest(delay_ms=delay_ms):
                result = calibration.estimate_delay(*dataset(delay_ms))
                self.assertAlmostEqual(result["delay_ms"], delay_ms, delta=0.25)
                self.assertLess(np.ptp(result["segment_delay_ms"]), 0.5)
                compensated = calibration.apply_time_compensation(result, -2_000_000, 3_000_000)
                self.assertAlmostEqual(compensated["runtime_delay_ms"], delay_ms + 1, delta=0.25)
                self.assertEqual(compensated["suggested_config"]["measurement_latency_ns"], 3_000_000)
                self.assertAlmostEqual(compensated["suggested_config"]["camera_timestamp_bias_ns"] / 1e6,
                                       -delay_ms - 3, delta=0.25)

    def test_rejects_unobservable_motion(self):
        camera_ns, rotations, imu_ns, gyro = dataset(0)
        rotations[:] = np.eye(3)
        gyro[:] = 0
        with self.assertRaises(calibration.CalibrationError):
            calibration.estimate_delay(camera_ns, rotations, imu_ns, gyro)

    def test_rejects_time_rollback_and_imu_gaps(self):
        for bad_input in ("rollback", "gap", "fractional"):
            camera_ns, rotations, imu_ns, gyro = dataset(3)
            if bad_input == "rollback":
                camera_ns[100] = camera_ns[99]
            elif bad_input == "gap":
                imu_ns[1000:] += 100_000_000
            else:
                camera_ns = camera_ns.astype(float)
            with self.subTest(bad_input=bad_input), self.assertRaises(calibration.CalibrationError):
                calibration.estimate_delay(camera_ns, rotations, imu_ns, gyro)

    def test_rejects_boundary_and_retains_unstable_segment_diagnostics(self):
        with self.assertRaises(calibration.CalibrationError):
            calibration.estimate_delay(*dataset(35), max_delay_ms=20)
        camera_ns, rotations, imu_ns, gyro = dataset(2)
        camera_ns[len(camera_ns) // 2:] += 18_000_000
        with self.assertRaises(calibration.CalibrationError) as caught:
            calibration.estimate_delay(camera_ns, rotations, imu_ns, gyro)
        self.assertIn("segment_delay_ms", caught.exception.diagnostics)
        self.assertGreater(caught.exception.diagnostics["segment_spread_ms"], 5)
        self.assertNotIn("suggested_config", caught.exception.diagnostics)

    def test_rejected_cli_report_retains_fit_and_load_diagnostics(self):
        camera_ns, rotations, imu_ns, gyro = dataset(2)
        camera_ns[len(camera_ns) // 2:] += 18_000_000
        with tempfile.TemporaryDirectory(prefix="mosas-sync-report-") as directory:
            arguments = (camera_ns, rotations, imu_ns, gyro)
            quality = {"runtime_timing": {"guidance": {"samples": 0}}}
            with patch.object(calibration, "load_dataset", return_value=(arguments, quality, (0, 0))), \
                    redirect_stderr(io.StringIO()):
                code = calibration.main(["--data", directory, "--intrinsics", "unused.json",
                                         "--columns", "9", "--rows", "6", "--square-size-m", "0.03"])
            self.assertEqual(code, 1)
            result = json.loads((Path(directory) / "calibration_result.json").read_text())
            self.assertFalse(result["valid"])
            self.assertIn("segment_delay_ms", result)
            self.assertIn("runtime_timing", result)
            self.assertNotIn("suggested_config", result)

    def test_preparation_matches_runtime_flip_and_resize(self):
        raw = np.arange(6, dtype=np.uint8).reshape(2, 3)
        expected = cv2.flip(cv2.resize(raw, (6, 4), interpolation=cv2.INTER_LINEAR), -1)
        np.testing.assert_array_equal(calibration.prepare_frame(raw, 6, 4), expected)

    def test_rejects_corrupt_or_incomplete_captures_before_pose_estimation(self):
        scenarios = ("incomplete", "version", "direction", "dimensions", "truncated", "offset",
                     "timestamp", "compensation", "symlink", "imu_loss", "count")
        for scenario in scenarios:
            with self.subTest(scenario=scenario), tempfile.TemporaryDirectory(prefix="mosas-sync-test-") as directory:
                root = Path(directory)
                metadata = capture_fixture(root)
                if scenario == "incomplete":
                    metadata["complete"] = False
                elif scenario == "version":
                    metadata["version"] = 1
                elif scenario in ("direction", "dimensions"):
                    intrinsics = json.loads((root / "intrinsics.json").read_text())
                    intrinsics["image_orientation" if scenario == "direction" else "image_width"] = (
                        "sdk_raw" if scenario == "direction" else 640)
                    (root / "intrinsics.json").write_text(json.dumps(intrinsics))
                elif scenario == "truncated":
                    with (root / "frames.gray8").open("r+b") as stream:
                        stream.truncate(20)
                elif scenario == "symlink":
                    outside = root / "external"
                    (root / "imu.csv").rename(outside)
                    (root / "imu.csv").symlink_to(outside)
                elif scenario in ("offset", "timestamp", "compensation"):
                    with (root / "frames.csv").open(newline="") as stream:
                        rows = list(csv.DictReader(stream))
                    rows[1][{"offset": "offset", "timestamp": "timestamp_ns",
                             "compensation": "runtime_timestamp_ns"}[scenario]] = rows[0][
                                 {"offset": "offset", "timestamp": "timestamp_ns",
                                  "compensation": "runtime_timestamp_ns"}[scenario]]
                    write_csv(root / "frames.csv", FRAME_FIELDS, [[row[key] for key in FRAME_FIELDS] for row in rows])
                    metadata["recording"]["data_bytes"] = sum((root / name).stat().st_size
                        for name in ("frames.gray8", "frames.csv", "imu.csv"))
                elif scenario == "imu_loss":
                    metadata["imu_source"]["dropped_sample_count"] = 1
                elif scenario == "count":
                    metadata["recording"]["camera_written"] += 1
                (root / "metadata.json").write_text(json.dumps(metadata))
                with self.assertRaises(calibration.CalibrationError):
                    calibration.load_dataset(root, root / "intrinsics.json", 9, 6, 0.03)

    def test_cli_from_upside_down_distorted_checkerboard(self):
        with tempfile.TemporaryDirectory(prefix="mosas-sync-image-test-") as directory:
            root = Path(directory)
            capture_fixture(root, rendered=True)
            process = subprocess.run(
                [sys.executable, "-B", str(SCRIPT), "--data", str(root),
                 "--intrinsics", str(root / "intrinsics.json"), "--columns", "9", "--rows", "6",
                 "--square-size-m", "0.03"], text=True, capture_output=True, timeout=60,
                env={**os.environ, "OPENCV_FOR_THREADS_NUM": "1"})
            self.assertEqual(process.returncode, 0, process.stderr)
            result = json.loads((root / "calibration_result.json").read_text())
            self.assertTrue(result["valid"])
            self.assertAlmostEqual(result["delay_ms"], 8, delta=2.5)
            self.assertAlmostEqual(result["runtime_delay_ms"], 9, delta=2.5)
            self.assertAlmostEqual(result["suggested_config"]["camera_timestamp_bias_ns"] / 1e6, -11, delta=2.5)
            self.assertIn("camera_callback_age_ms", result["data_quality"])
            self.assertEqual(result["runtime_timing"]["guidance"]["samples"], 0)
            self.assertIn("TX", result["limitations"])


if __name__ == "__main__":
    unittest.main()
