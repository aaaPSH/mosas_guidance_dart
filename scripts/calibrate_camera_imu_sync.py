#!/usr/bin/env python3
"""@brief 使用原程序生成的 version-2 采集数据，离线标定相机与 IMU 的相对时延。

SDK 原方向灰度图先按正常处理尺寸缩放并旋转 180 度，再应用用户的正立图像内参。
依赖 NumPy 和 OpenCV，不修改生产配置；主机接收时间估计不能提供绝对硬件同步。
"""

import argparse
import csv
import json
from pathlib import Path
import re
import sys
import tempfile

import cv2
import numpy as np

INT64_MAX = (1 << 63) - 1
FRAME_FIELDS = ["frame_index", "runtime_timestamp_ns", "timestamp_ns", "received_timestamp_ns",
                "observed_timestamp_ns", "timestamp_source", "offset", "length", "width", "height"]
IMU_FIELDS = ["sample_index", "runtime_timestamp_ns", "timestamp_ns", "observed_timestamp_ns",
              "acceleration_x_mps2", "acceleration_y_mps2", "acceleration_z_mps2",
              "angular_velocity_x_rad_s", "angular_velocity_y_rad_s", "angular_velocity_z_rad_s",
              "servo_control_p_normalized", "servo_control_y_normalized"]


class CalibrationError(ValueError):
    """@brief 表示数据或估计被拒绝，并保留拒绝前已计算出的诊断。"""

    def __init__(self, message, diagnostics=None):
        """@param message 拒绝原因。 @param diagnostics 已获得的、可写入 JSON 的部分诊断。"""
        super().__init__(message)
        self.diagnostics = dict(diagnostics or {})


def _integer(value, name, minimum=0, maximum=INT64_MAX):
    """@brief 严格解析有界整数，不截断浮点时间戳，也不将布尔值当成整数。"""
    if isinstance(value, str):
        if re.fullmatch(r"-?[0-9]+", value) is None:
            raise CalibrationError(f"{name}: 须为十进制整数")
        value = int(value)
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise CalibrationError(f"{name}: 须为整数，禁止浮点时间戳")
    value = int(value)
    if not minimum <= value <= maximum:
        raise CalibrationError(f"{name}: 数值超出允许范围")
    return value


def _increasing(timestamps, name):
    """@brief 按记录顺序校验精确的非负纳秒时间，不静默排序或修复数据。"""
    values = np.asarray([_integer(value, name) for value in timestamps], dtype=np.int64)
    if values.ndim != 1 or len(values) < 2 or np.any(np.diff(values) <= 0):
        raise CalibrationError(f"{name}: 时间戳必须非负且严格递增，且至少两条记录")
    return values


def _quaternion_product(left, right):
    """@brief 组合标量在前的单位四元数，采用机体系到世界系的旋转约定。"""
    return np.r_[left[0] * right[0] - left[1:] @ right[1:],
                 left[0] * right[1:] + right[0] * left[1:] + np.cross(left[1:], right[1:])]


def _integrate_gyro(times, gyro):
    """@brief 使用中点 SO(3) 增量积分已扣除零偏的机体系角速度。

    @param times IMU 时间轴上严格递增的秒数。
    @param gyro 机体系角速度，单位 rad/s，形状为 (N, 3)。
    @return 各 IMU 时刻连续的单位四元数。
    """
    orientation = np.empty((len(times), 4))
    orientation[0] = [1, 0, 0, 0]
    increments = 0.5 * (gyro[:-1] + gyro[1:]) * np.diff(times)[:, None]
    for index, vector in enumerate(increments):
        angle = np.linalg.norm(vector)
        scale = np.sin(angle / 2) / angle if angle > 1e-12 else 0.5
        step = np.r_[np.cos(angle / 2), scale * vector]
        value = _quaternion_product(orientation[index], step)
        orientation[index + 1] = value / np.linalg.norm(value)
    return orientation


def _interpolate_orientation(times, orientation, queries):
    """@brief 仅在已记录的 IMU 历史内做四元数球面插值，禁止外推。"""
    if np.any(queries < times[0]) or np.any(queries > times[-1]):
        raise CalibrationError("查询超出 IMU 历史，禁止外推")
    indices = np.clip(np.searchsorted(times, queries, side="right") - 1, 0, len(times) - 2)
    fraction = (queries - times[indices]) / (times[indices + 1] - times[indices])
    lower, upper = orientation[indices], orientation[indices + 1].copy()
    dot = np.sum(lower * upper, axis=1)
    upper[dot < 0] *= -1
    angle = np.arccos(np.clip(np.abs(dot), 0, 1))
    denominator = np.sin(angle)
    small = denominator < 1e-8
    denominator[small] = 1
    weight0 = np.sin((1 - fraction) * angle) / denominator
    weight1 = np.sin(fraction * angle) / denominator
    weight0[small], weight1[small] = 1 - fraction[small], fraction[small]
    result = weight0[:, None] * lower + weight1[:, None] * upper
    return result / np.linalg.norm(result, axis=1)[:, None]


def estimate_delay(camera_time_ns, rotations, imu_time_ns, gyro_rad_s,
                   max_delay_ms=100.0, stationary_seconds=2.0,
                   max_segment_spread_ms=5.0):
    """@brief 在相同时间区间内匹配不依赖坐标系方向的视觉旋转量与陀螺仪积分量。

    @param camera_time_ns 未补偿的 CLOCK_MONOTONIC 相机时间戳，单位 ns。
    @param rotations 棋盘格到相机的旋转矩阵，形状为 (N, 3, 3)。
    @param imu_time_ns 未补偿的 IMU 串口接收时间估计，单位 ns。
    @param gyro_rad_s 原始机体系角速度，单位 rad/s；使用起始静止段估计零偏。
    @param max_delay_ms 对称搜索范围，单位 ms；最优值落在边界时拒绝。
    @param stationary_seconds 起始静止段时长，至少 0.2 秒。
    @param max_segment_spread_ms 三个分段时延估计的最大允许跨度，单位 ms。
    @return 含有效标志、原始相对时延和诊断的字典；拒绝时抛出携带部分诊断的 CalibrationError。
    @note 正时延表示同一运动的相机时间戳较晚；幅值匹配不要求或估计安装外参及绝对采样时刻。
    """
    camera_ns = _increasing(camera_time_ns, "相机")
    imu_ns = _increasing(imu_time_ns, "IMU")
    rotations = np.asarray(rotations, dtype=float)
    gyro = np.asarray(gyro_rad_s, dtype=float)
    parameters = np.array([max_delay_ms, stationary_seconds, max_segment_spread_ms])
    if (not np.all(np.isfinite(parameters)) or not 1 <= max_delay_ms <= 500 or
            stationary_seconds < 0.2 or max_segment_spread_ms <= 0):
        raise CalibrationError("搜索范围须为 1～500 ms，静止段须至少 0.2 s，分段容差须为正")
    if (rotations.shape != (len(camera_ns), 3, 3) or gyro.shape != (len(imu_ns), 3) or
            not np.all(np.isfinite(rotations)) or not np.all(np.isfinite(gyro))):
        raise CalibrationError("旋转矩阵或 IMU 数据形状错误/包含非有限值")
    if (not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=1e-5) or
            not np.allclose(np.linalg.det(rotations), 1, atol=1e-5)):
        raise CalibrationError("相机姿态不是有效的旋转矩阵")
    origin = int(imu_ns[0])
    camera_time = (camera_ns - origin).astype(float) * 1e-9
    imu_time = (imu_ns - origin).astype(float) * 1e-9
    if len(camera_ns) < 100 or imu_time[-1] < 8 or camera_time[-1] - camera_time[0] < 8:
        raise CalibrationError("有效数据不足：需要至少 100 个棋盘格观测和 8 秒连续采集")
    if np.max(np.diff(imu_time)) > 0.025:
        raise CalibrationError("IMU 存在超过 25 ms 的采样间断，请重新采集")
    stationary = gyro[imu_time <= stationary_seconds]
    if len(stationary) < 20 or np.linalg.norm(np.std(stationary, axis=0)) > 0.03:
        raise CalibrationError("起始静止段不稳定；开始后请保持整机静止至少 3 秒")
    bias = np.mean(stationary, axis=0)
    if np.linalg.norm(bias) > 0.1:
        raise CalibrationError("起始段并未静止或陀螺仪单位不是 rad/s")
    orientation = _integrate_gyro(imu_time, gyro - bias)
    diagnostics = {"gyro_bias_rad_s": bias.tolist()}

    # 比较实际约 60 ms 的积分区间，避免直接差分引入半帧时间偏移。
    first = np.arange(len(camera_time))
    second = np.searchsorted(camera_time, camera_time + 0.060)
    valid = second < len(camera_time)
    first, second = first[valid], second[valid]
    duration = camera_time[second] - camera_time[first]
    search_seconds = max_delay_ms / 1000
    valid = ((duration <= 0.15) &
             (camera_time[first] - search_seconds >= imu_time[0]) &
             (camera_time[second] + search_seconds <= imu_time[-1]))
    first, second, duration = first[valid], second[valid], duration[valid]
    cosine = (np.einsum("nij,nij->n", rotations[first], rotations[second]) - 1) / 2
    camera_rate = np.arccos(np.clip(cosine, -1, 1)) / duration
    valid = camera_rate < 8.0  # 剔除异常高角速率，避免角点顺序或 PnP 翻转干扰。
    first, second, duration, camera_rate = first[valid], second[valid], duration[valid], camera_rate[valid]
    diagnostics["rotation_intervals"] = len(first)
    if len(first) < 80 or np.std(camera_rate) < 0.06 or np.ptp(camera_rate) < 0.2:
        raise CalibrationError("转动激励不足：请做变速、多轴往复转动，保持棋盘格可见", diagnostics)
    starts, ends = camera_time[first], camera_time[second]

    def predicted_rate(delay_ms):
        lower = _interpolate_orientation(imu_time, orientation, starts - delay_ms / 1000)
        upper = _interpolate_orientation(imu_time, orientation, ends - delay_ms / 1000)
        cosine = np.clip(np.abs(np.sum(lower * upper, axis=1)), 0, 1)
        return 2 * np.arccos(cosine) / duration

    def score(delay_ms, mask):
        residual = np.abs((predicted_rate(delay_ms) - camera_rate)[mask])
        # 以 0.05 rad/s 为转折点的 Huber 损失，降低少量姿态异常对偏移搜索的影响。
        loss = np.where(residual <= 0.05, residual ** 2, 0.1 * residual - 0.0025)
        return float(np.sqrt(np.mean(loss)))

    grid = np.linspace(-max_delay_ms, max_delay_ms, int(np.ceil(2 * max_delay_ms)) + 1)

    def optimize(mask):
        costs = np.array([score(value, mask) for value in grid])
        best = int(np.argmin(costs))
        if best == 0 or best == len(grid) - 1:
            raise CalibrationError("最优偏移位于搜索边界：扩大搜索范围或检查数据", diagnostics)
        # 先粗网格定位候选，再仅在其邻域做黄金分割搜索，不假定整个搜索范围单峰。
        left, right = grid[best - 1], grid[best + 1]
        ratio = (np.sqrt(5) - 1) / 2
        x1, x2 = right - ratio * (right - left), left + ratio * (right - left)
        f1, f2 = score(x1, mask), score(x2, mask)
        for _ in range(24):
            if f1 < f2:
                right, x2, f2 = x2, x1, f1
                x1 = right - ratio * (right - left)
                f1 = score(x1, mask)
            else:
                left, x1, f1 = x1, x2, f2
                x2 = left + ratio * (right - left)
                f2 = score(x2, mask)
        return float((left + right) / 2), costs

    all_intervals = np.ones(len(starts), dtype=bool)
    delay_ms, costs = optimize(all_intervals)
    diagnostics.update(delay_ms=delay_ms, definition="camera_raw_timestamp - imu_raw_timestamp for the same motion")
    alternatives = costs[np.abs(grid - delay_ms) >= 15]
    if len(alternatives) and np.min(alternatives) <= score(delay_ms, all_intervals) * 1.05 + 0.0005:
        raise CalibrationError("时间偏移不明确：存在相近的匹配结果，请增加非周期变速运动", diagnostics)
    predicted = predicted_rate(delay_ms)
    rmse = float(np.sqrt(np.mean((predicted - camera_rate) ** 2)))
    correlation = float(np.corrcoef(predicted, camera_rate)[0, 1])
    diagnostics.update(rmse_rad_s=rmse, correlation=correlation if np.isfinite(correlation) else None)
    if not np.isfinite(correlation) or correlation < 0.85 or rmse > max(0.08, 0.3 * np.std(camera_rate)):
        raise CalibrationError("图像与陀螺仪运动不一致：检查内参、单位、刚性安装及棋盘格是否静止", diagnostics)
    centers = (starts + ends) / 2
    boundaries = np.linspace(centers[0], centers[-1] + 1e-9, 4)
    diagnostics["segment_delay_ms"] = []
    for low, high in zip(boundaries[:-1], boundaries[1:]):
        mask = (centers >= low) & (centers < high)
        if np.count_nonzero(mask) < 25 or np.std(camera_rate[mask]) < 0.05:
            raise CalibrationError("分段运动不足：请在整段采集中持续做变速转动", diagnostics)
        diagnostics["segment_delay_ms"].append(optimize(mask)[0])
    diagnostics["segment_spread_ms"] = float(np.ptp(diagnostics["segment_delay_ms"]))
    if diagnostics["segment_spread_ms"] > max_segment_spread_ms:
        raise CalibrationError(f"分段偏移不稳定 ({diagnostics['segment_spread_ms']:.2f} ms)，固定补偿无法解释", diagnostics)
    return {"valid": True, **diagnostics}


def apply_time_compensation(result, camera_bias_ns, imu_latency_ns):
    """@brief 解释原始时延估计，保留现有 IMU 补偿，仅建议相机补偿值。

    @param result estimate_delay 已接受的估计结果。
    @param camera_bias_ns 配置中的有符号相机时间补偿，单位 ns。
    @param imu_latency_ns 配置中的非负 IMU 接收时间补偿，单位 ns。
    @return 扩展诊断及建议相机补偿；不修改生产配置。
    """
    camera_bias_ns = _integer(camera_bias_ns, "camera bias", -INT64_MAX - 1)
    imu_latency_ns = _integer(imu_latency_ns, "IMU latency")
    if result.get("valid") is not True:
        raise CalibrationError("不得为拒绝的标定结果生成补偿值", result)
    new_bias = _integer(-int(round(result["delay_ms"] * 1e6)) - imu_latency_ns,
                        "建议相机偏置", -INT64_MAX - 1)
    return {**result, "runtime_delay_ms": result["delay_ms"] + (camera_bias_ns + imu_latency_ns) / 1e6,
            "current_config": {"camera_timestamp_bias_ns": camera_bias_ns, "measurement_latency_ns": imu_latency_ns},
            "camera_bias_adjustment_ns": new_bias - camera_bias_ns,
            "suggested_config": {"camera_timestamp_bias_ns": new_bias, "measurement_latency_ns": imu_latency_ns}}


def prepare_frame(raw, width, height):
    """@brief 在离线图像上复现 Nori prepare 的几何处理。

    @param raw 尚未执行运行时原地翻转的 SDK 原方向灰度图。
    @param width 正立内参对应的正常处理图像宽度。
    @param height 正立内参对应的正常处理图像高度。
    @return 独立翻转后的正立图像，不改变存储像素及时间戳。
    """
    if raw.shape != (height, width):
        raw = cv2.resize(raw, (width, height), interpolation=cv2.INTER_LINEAR)
    return cv2.flip(raw, -1)


def _capture_file(root, name):
    """@brief 要求固定名称的采集文件为采集目录内的普通文件，拒绝符号链接。"""
    path = root / name
    if path.is_symlink() or not path.is_file() or path.resolve().parent != root:
        raise CalibrationError(f"采集文件缺失、非普通文件或路径越界: {name}")
    return path


def _read_json(path):
    """@brief 拒绝 JSON 中的 NaN、Infinity 或非有限溢出值，包括诊断字段。"""
    def invalid(token):
        raise CalibrationError(f"JSON 包含非有限值: {token}")
    def finite_float(token):
        value = float(token)
        if not np.isfinite(value):
            raise CalibrationError("JSON 数值溢出为非有限值")
        return value
    value = json.loads(path.read_text(encoding="utf-8"), parse_constant=invalid, parse_float=finite_float)
    if not isinstance(value, dict):
        raise CalibrationError("JSON 顶层必须是对象")
    return value


def _read_rows(path, fields):
    """@brief 接受带索引的记录前，严格检查 CSV 列名及行完整性。"""
    with path.open(newline="", encoding="ascii") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != fields:
            raise CalibrationError(f"{path.name}: CSV 列与 version=2 不一致")
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise CalibrationError(f"{path.name}: CSV 行不完整")
            yield row


def _distribution_ms(values):
    """@brief 计算数据可用延迟及间隔的经验分位数，不将其解释为采样时间误差。"""
    values = np.asarray(values, dtype=np.float64) / 1e6
    if not len(values) or not np.all(np.isfinite(values)):
        raise CalibrationError("诊断时间间隔无效")
    return {"samples": len(values), "min": float(np.min(values)), "mean": float(np.mean(values)),
            "p50": float(np.percentile(values, 50)), "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)), "max": float(np.max(values))}


def load_dataset(directory, intrinsics_path, columns, rows, square_size_m):
    """@brief 校验完整采集，并使用正立图像内参恢复视觉旋转。

    @param directory 包含 version-2 元数据、灰度帧和原始 IMU 的采集目录。
    @param intrinsics_path 声明 runtime_prepared_180 方向、内参矩阵、畸变和尺寸的 JSON 文件。
    @param columns 棋盘格内角点列数，不是方格数量。
    @param rows 棋盘格内角点行数。
    @param square_size_m 棋盘格单格边长，单位 m。
    @return estimate_delay 的参数组、数据及姿态诊断、采集时使用的时间补偿。
    """
    root = Path(directory).resolve(strict=True)
    metadata = _read_json(_capture_file(root, "metadata.json"))
    intrinsics = _read_json(Path(intrinsics_path))
    if (metadata.get("version") != 2 or metadata.get("complete") is not True or
            metadata.get("end_reason") != "duration_elapsed" or metadata.get("recording_error") != "none" or
            metadata.get("control_output_disabled") is not True or
            metadata.get("image_orientation") != "sdk_raw" or
            metadata.get("analysis_image_orientation") != "runtime_prepared_180" or
            metadata.get("prepare_rotation_deg") != 180 or
            metadata.get("imu_timestamp_source") != "sequence_fixed_period" or
            metadata.get("imu_timestamp_quality") != "estimated"):
        raise CalibrationError("采集未完成、格式/方向不支持或不是接收时间估计的只读标定模式")
    if metadata.get("camera_clock_domain") not in ("monotonic", "realtime"):
        raise CalibrationError("相机时钟域不明确")
    bias = _integer(metadata["camera_timestamp_bias_ns"], "camera bias", -INT64_MAX - 1)
    latency = _integer(metadata["measurement_latency_ns"], "IMU latency")
    width = _integer(metadata["prepared_width"], "prepared width", 1, 16384)
    height = _integer(metadata["prepared_height"], "prepared height", 1, 16384)
    raw_width = _integer(metadata["raw_width"], "raw width", 1, 16384)
    raw_height = _integer(metadata["raw_height"], "raw height", 1, 16384)
    if (intrinsics.get("image_orientation") != "runtime_prepared_180" or
            intrinsics.get("image_width") != width or intrinsics.get("image_height") != height):
        raise CalibrationError("内参须来自翻转后正立图像，且与 prepare 输出分辨率一致")
    matrix = np.asarray(intrinsics["camera_matrix"], dtype=np.float64)
    distortion = np.asarray(intrinsics["distortion_coefficients"], dtype=np.float64).ravel()
    if (matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)) or
            matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1]) or
            len(distortion) not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(distortion))):
        raise CalibrationError("内参/畸变参数无效；畸变为零也须显式提供")
    columns = _integer(columns, "内角点列数", 3, 30)
    rows = _integer(rows, "内角点行数", 3, 30)
    if not np.isfinite(square_size_m) or not 0 < square_size_m <= 1:
        raise CalibrationError("棋盘格格子边长须为 0～1 米之间的正数")
    recording = metadata["recording"]
    imu_source = metadata["imu_source"]
    if (imu_source.get("available") is not True or
            any(_integer(imu_source[name], name) for name in (
                "parse_error_count", "dropped_chunk_count", "sequence_gap_count",
                "crc_error_count", "dropped_sample_count")) or
            _integer(recording["camera_dropped"], "camera dropped") or
            _integer(recording["imu_dropped"], "IMU dropped") or
            _integer(metadata["camera"]["timestamp_dropped"], "camera timestamp dropped")):
        raise CalibrationError("采集存在 IMU 丢失/解码错误、时间戳错误或标定记录溢出")
    files = {name: _capture_file(root, name) for name in ("frames.gray8", "frames.csv", "imu.csv")}
    if sum(path.stat().st_size for path in files.values()) != _integer(recording["data_bytes"], "data bytes"):
        raise CalibrationError("采集文件长度与完成记录不一致")
    frame_rows, camera_all, received, observed = [], [], [], []
    offset = 0
    expected_source = "sdk_monotonic" if metadata["camera_clock_domain"] == "monotonic" else "sdk_realtime_converted"
    for index, row in enumerate(_read_rows(files["frames.csv"], FRAME_FIELDS)):
        frame_index = _integer(row["frame_index"], "frame index")
        timestamp = _integer(row["timestamp_ns"], "camera timestamp")
        runtime_ns = _integer(row["runtime_timestamp_ns"], "camera runtime timestamp")
        callback_ns = _integer(row["received_timestamp_ns"], "callback timestamp")
        observed_ns = _integer(row["observed_timestamp_ns"], "camera observed timestamp")
        length = _integer(row["length"], "frame length", 1)
        if (frame_index != index or _integer(row["offset"], "frame offset") != offset or
                length != raw_width * raw_height or
                _integer(row["width"], "frame width") != raw_width or
                _integer(row["height"], "frame height") != raw_height or
                offset + length > files["frames.gray8"].stat().st_size or
                runtime_ns - bias != timestamp or row["timestamp_source"] != expected_source or
                observed_ns < callback_ns):
            raise CalibrationError("图像索引、几何、时间补偿或字节范围不一致")
        frame_rows.append((offset, length))
        offset += length
        camera_all.append(timestamp)
        received.append(callback_ns)
        observed.append(observed_ns)
    camera_all = _increasing(camera_all, "相机原始记录")
    if (offset != files["frames.gray8"].stat().st_size or offset != _integer(recording["pixel_bytes"], "pixel bytes") or
            len(frame_rows) != _integer(recording["camera_written"], "camera written") or
            len(frame_rows) != _integer(recording["camera_selected"], "camera selected") or
            _integer(recording["camera_seen"], "camera seen") < len(frame_rows) or np.any(np.diff(observed) < 0)):
        raise CalibrationError("图像数量、总长度或交付时间不一致")
    imu_times, imu_observed, gyro = [], [], []
    for index, row in enumerate(_read_rows(files["imu.csv"], IMU_FIELDS)):
        timestamp = _integer(row["timestamp_ns"], "IMU timestamp")
        runtime_ns = _integer(row["runtime_timestamp_ns"], "IMU runtime timestamp")
        if _integer(row["sample_index"], "sample index") != index or runtime_ns + latency != timestamp:
            raise CalibrationError("IMU 序号或时间补偿不一致")
        values = np.array([float(row[name]) for name in IMU_FIELDS[4:]])
        if not np.all(np.isfinite(values)):
            raise CalibrationError("IMU 数据包含非有限值")
        imu_times.append(timestamp)
        imu_observed.append(_integer(row["observed_timestamp_ns"], "IMU observed timestamp"))
        gyro.append(values[3:6])
    imu_times = _increasing(imu_times, "IMU 原始记录")
    if (len(imu_times) != _integer(recording["imu_written"], "IMU written") or
            len(imu_times) != _integer(recording["imu_accepted"], "IMU accepted") or np.any(np.diff(imu_observed) < 0)):
        raise CalibrationError("IMU 数量或交付时间不一致")
    quality = {
        "data_quality": {
            "camera_interval_ms": _distribution_ms(np.diff(camera_all)),
            "imu_interval_ms": _distribution_ms(np.diff(imu_times)),
            "camera_callback_age_ms": _distribution_ms(np.asarray(received, dtype=np.int64) - camera_all),
            "camera_available_delay_ms": _distribution_ms(np.asarray(observed, dtype=np.int64) - camera_all),
            "imu_available_delay_ms": _distribution_ms(np.asarray(imu_observed, dtype=np.int64) - imu_times),
            "definition": "Availability/interval statistics, NOT absolute sampling-time uncertainty.",
        }, "recording": recording, "imu_source": imu_source, "camera": metadata["camera"],
        "runtime_timing": metadata.get("runtime_timing", {}), "runtime_fps": metadata.get("runtime_fps", {}),
        "limitations": "Relative fixed delay only; no hardware sync. Actual bench branches only; TX disabled; recording adds load.",
    }
    object_points = np.zeros((columns * rows, 3), np.float64)
    object_points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size_m
    camera_times, rotations, errors = [], [], []
    with files["frames.gray8"].open("rb") as stream:
        for timestamp, (offset, length) in zip(camera_all, frame_rows):
            stream.seek(offset)
            data = stream.read(length)
            if len(data) != length:
                raise CalibrationError("图像读取不完整", quality)
            raw = np.frombuffer(data, np.uint8).reshape(raw_height, raw_width)
            prepared = prepare_frame(raw, width, height)
            found, corners = cv2.findChessboardCorners(prepared, (columns, rows), flags=(
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK))
            if not found:
                continue
            cv2.cornerSubPix(prepared, corners, (5, 5), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
            ok, vector, translation = cv2.solvePnP(object_points, corners, matrix, distortion)
            if not ok:
                continue
            rotation = cv2.Rodrigues(vector)[0]
            if np.min((object_points @ rotation.T + translation.ravel())[:, 2]) <= 0:
                continue
            projected, _ = cv2.projectPoints(object_points, vector, translation, matrix, distortion)
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1))))
            if np.isfinite(error) and error <= 0.75:
                camera_times.append(int(timestamp))
                rotations.append(rotation)
                errors.append(error)
    quality.update(accepted_poses=len(rotations), rejected_poses=len(frame_rows) - len(rotations),
                   pose_acceptance_rate=len(rotations) / len(frame_rows),
                   mean_reprojection_error_px=float(np.mean(errors)) if errors else None)
    return (camera_times, rotations, imu_times, gyro), quality, (bias, latency)


def _write_report(directory, result):
    """@brief 仅原子替换既有采集目录中的结果报告，不改写原始采集文件。"""
    root = Path(directory).resolve(strict=True)
    target = root / "calibration_result.json"
    if target.is_symlink():
        raise CalibrationError("标定报告路径不得为符号链接")
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", prefix=".calibration-result-",
                                         dir=root, delete=False) as output:
            temporary = Path(output.name)
            output.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        temporary.replace(target)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main(argv=None):
    """@brief 离线分析应用采集数据，不改变生产配置。

    @param argv 可选命令行参数列表；省略时使用进程参数。
    @return 接受估计并写入报告时返回 0，拒绝数据或报告失败返回 1，命令行错误返回 2。
    """
    parser = argparse.ArgumentParser(description="真实程序负载下的棋盘格/IMU 相对时延标定，不修改生产配置")
    parser.add_argument("--data", type=Path, required=True, help="标定配置 sync_calibration.output_dir 指定的采集目录")
    parser.add_argument("--intrinsics", type=Path, required=True, help="翻转 180°后正立图像的内参/畸变 JSON")
    parser.add_argument("--columns", type=int, required=True, help="棋盘格内角点列数")
    parser.add_argument("--rows", type=int, required=True, help="棋盘格内角点行数")
    parser.add_argument("--square-size-m", type=float, required=True, help="单格边长，米")
    parser.add_argument("--max-delay-ms", type=float, default=100.0)
    parser.add_argument("--stationary-seconds", type=float, default=2.0)
    parser.add_argument("--max-segment-spread-ms", type=float, default=5.0)
    options = parser.parse_args(argv)
    if (not np.all(np.isfinite([options.max_delay_ms, options.stationary_seconds, options.max_segment_spread_ms])) or
            not 1 <= options.max_delay_ms <= 500 or options.stationary_seconds < 0.2 or
            options.max_segment_spread_ms <= 0 or not 3 <= options.columns <= 30 or not 3 <= options.rows <= 30 or
            not np.isfinite(options.square_size_m) or not 0 < options.square_size_m <= 1):
        parser.error("棋盘格规格或搜索/静止/分段容差参数无效")
    quality = {}
    try:
        arguments, quality, corrections = load_dataset(options.data, options.intrinsics,
                                                       options.columns, options.rows, options.square_size_m)
        result = estimate_delay(*arguments, max_delay_ms=options.max_delay_ms,
                                stationary_seconds=options.stationary_seconds,
                                max_segment_spread_ms=options.max_segment_spread_ms)
        result = {**quality, **apply_time_compensation(result, *corrections)}
    except (CalibrationError, OSError, ValueError, KeyError, TypeError, OverflowError, csv.Error, cv2.error) as error:
        result = {**quality, **getattr(error, "diagnostics", {}), "valid": False, "reason": str(error)}
        result.pop("suggested_config", None)
    try:
        _write_report(options.data, result)
    except (OSError, ValueError, TypeError) as error:
        print(f"无法写入标定报告: {error}", file=sys.stderr)
        return 1
    if not result["valid"]:
        print(f"标定拒绝: {result['reason']}", file=sys.stderr)
        return 1
    print(f"原始相对偏移 {result['delay_ms']:+.3f} ms；当前补偿后剩余 {result['runtime_delay_ms']:+.3f} ms")
    print(f"分段跨度 {result['segment_spread_ms']:.3f} ms（不是逐样本同步误差）")
    for key, value in result["suggested_config"].items():
        print(f"{key}={value}")
    print("仅为本次台架负载下的相对固定补偿；控制 TX 已禁用，记录增加开销，原配置未修改。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
