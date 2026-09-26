#!/usr/bin/env python3
"""录制视频视觉流程回放与参数调节工具。"""

from __future__ import annotations

import argparse
import configparser
import csv
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "mosas_guidance.conf"


@dataclass
class VisionParams:
    """与 C++ VisionConfig 对齐的可调参数。"""

    initial_roi_x: int = 0
    initial_roi_y: int = 0
    initial_roi_width: int = 640
    initial_roi_height: int = 480
    h_min: int = 35
    h_max: int = 85
    s_min: int = 80
    s_max: int = 255
    v_min: int = 100
    v_max: int = 255
    found_range_x: int = 40
    found_range_y: int = 40
    min_blob_area: int = 8
    min_aspect_ratio: float = 0.5
    min_fill_ratio: float = 0.35

    def initial_roi(self) -> tuple[int, int, int, int]:
        """返回尚未裁剪的初始 ROI。"""
        return (
            self.initial_roi_x,
            self.initial_roi_y,
            self.initial_roi_width,
            self.initial_roi_height,
        )


@dataclass
class VisionBlob:
    """已识别目标的位置和大小。"""

    x: int = 0
    y: int = 0
    width: int = 0
    height: int = 0
    area: int = 0
    center_x: float = 0.0
    center_y: float = 0.0


@dataclass
class PipelineResult:
    """单帧目标识别结果。"""

    found: bool = False
    roi: tuple[int, int, int, int] = (0, 0, 0, 0)
    next_roi: tuple[int, int, int, int] = (0, 0, 0, 0)
    blob: VisionBlob = field(default_factory=VisionBlob)


@dataclass(frozen=True)
class OverlayMask:
    """录像画面中旧可视化的遮挡范围，坐标使用原始视频像素。"""

    top_left_width: int
    top_left_height: int
    bottom_height: int
    reference_x: int
    reference_y: int
    reference_radius: int


@dataclass(frozen=True)
class RecordedDetection:
    """录像同名 CSV 中原识别器的目标记录。"""

    found: bool
    x: int
    y: int
    width: int
    height: int


def load_recorded_detections(video_path: Path) -> dict[int, RecordedDetection]:
    """按帧号读取同名 CSV；没有 CSV 时只进行视频回放。"""
    csv_path = video_path.with_suffix(".csv")
    if not csv_path.is_file():
        return {}
    detections: dict[int, RecordedDetection] = {}
    with csv_path.open("r", encoding="utf-8", newline="") as source:
        reader = csv.DictReader(source)
        required = {
            "frame_index", "vision_found", "target_x", "target_y",
            "target_width", "target_height",
        }
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"录像 CSV 缺少目标字段: {csv_path}")
        for row in reader:
            index = int(row["frame_index"])
            detections[index] = RecordedDetection(
                bool(int(row["vision_found"])),
                int(row["target_x"]),
                int(row["target_y"]),
                int(row["target_width"]),
                int(row["target_height"]),
            )
    return detections


def recorded_target_is_masked(
    detection: RecordedDetection, mask: OverlayMask,
    frame_width: int, frame_height: int,
) -> bool:
    """判断旧目标框是否与任一遮挡区相交。"""
    if not detection.found or detection.width <= 0 or detection.height <= 0:
        return False
    left, top = detection.x, detection.y
    right = left + detection.width - 1
    bottom = top + detection.height - 1
    if right < 0 or bottom < 0 or left >= frame_width or top >= frame_height:
        return False
    if mask.top_left_width > 0 and mask.top_left_height > 0:
        if left < mask.top_left_width and top < mask.top_left_height and \
                right >= 0 and bottom >= 0:
            return True
    if mask.bottom_height > 0 and bottom >= frame_height - mask.bottom_height:
        return True
    if mask.reference_radius > 0:
        nearest_x = max(left, min(mask.reference_x, right))
        nearest_y = max(top, min(mask.reference_y, bottom))
        return ((nearest_x - mask.reference_x) ** 2 +
                (nearest_y - mask.reference_y) ** 2 <=
                mask.reference_radius ** 2)
    return False


def mask_recorded_overlays(frame: np.ndarray, mask: OverlayMask) -> np.ndarray:
    """复制录像帧并遮挡已写入的文字与基准圆，供识别器使用。"""
    recognition_frame = frame.copy()
    height, width = recognition_frame.shape[:2]
    top_width = max(0, min(mask.top_left_width, width))
    top_height = max(0, min(mask.top_left_height, height))
    recognition_frame[:top_height, :top_width] = 0
    bottom_height = max(0, min(mask.bottom_height, height))
    if bottom_height:
        recognition_frame[height - bottom_height :] = 0
    if mask.reference_radius > 0:
        cv2.circle(
            recognition_frame,
            (mask.reference_x, mask.reference_y),
            mask.reference_radius,
            (0, 0, 0),
            -1,
        )
    return recognition_frame


def detect_recorded_status_marker(frame: np.ndarray) -> Optional[bool]:
    """读取录像左下角绿/红状态字；无明确字样时返回 None。"""
    if frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0:
        return None
    height, width = frame.shape[:2]
    marker = frame[max(0, height - 22) : max(1, height - 3), : min(width, 180)]
    if marker.size == 0:
        return None
    hsv = cv2.cvtColor(marker, cv2.COLOR_BGR2HSV)
    green = cv2.countNonZero(
        cv2.inRange(hsv, np.array((35, 60, 60), np.uint8),
                    np.array((90, 255, 255), np.uint8))
    )
    red_low = cv2.countNonZero(
        cv2.inRange(hsv, np.array((0, 60, 60), np.uint8),
                    np.array((12, 255, 255), np.uint8))
    )
    red_high = cv2.countNonZero(
        cv2.inRange(hsv, np.array((168, 60, 60), np.uint8),
                    np.array((179, 255, 255), np.uint8))
    )
    red = red_low + red_high
    if green >= 80 and green >= red * 1.5:
        return True
    if red >= 80 and red >= green * 1.5:
        return False
    return None


def load_overlay_mask(
    config_path: Path, frame_width: int, frame_height: int, min_blob_area: int
) -> OverlayMask:
    """根据现有可视化布局及相机主点生成录像遮挡默认值。"""
    parser = configparser.ConfigParser(interpolation=None)
    if not parser.read(config_path, encoding="utf-8"):
        raise ValueError(f"无法读取配置文件: {config_path}")
    camera = parser["camera"] if parser.has_section("camera") else {}
    center_x = float(camera.get("matrix_m02", frame_width / 2))
    center_y = float(camera.get("matrix_m12", frame_height / 2))
    if not math.isfinite(center_x) or not math.isfinite(center_y):
        raise ValueError("相机主点坐标必须是有限数值")
    radius = math.ceil(math.sqrt(max(0, min_blob_area) / math.pi)) + 3
    return OverlayMask(
        min(frame_width, 340),
        min(frame_height, 100),
        min(frame_height, 26),
        round(center_x),
        round(center_y),
        radius,
    )


class ReplaySession:
    """顺序处理录像；回退或改参数时从首帧重建跟踪状态。"""

    def __init__(
        self, capture: cv2.VideoCapture, params: VisionParams, mask: OverlayMask
    ):
        self.capture = capture
        self.params = params
        self.mask = mask
        self.pipeline = VisionReplayPipeline(params)
        self.frame_index = -1
        self.current_frame: Optional[np.ndarray] = None
        self.current_result: Optional[PipelineResult] = None
        self.current_recorded_marker: Optional[bool] = None
        self.history_found: list[bool] = []
        self._rewind()

    def _rewind(self) -> None:
        if not self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0):
            raise OSError("无法将视频回退到第一帧")
        self.pipeline = VisionReplayPipeline(self.params)
        self.frame_index = -1
        self.current_frame = None
        self.current_result = None
        self.current_recorded_marker = None
        self.history_found.clear()

    def update_params(self, params: VisionParams) -> None:
        if params != self.params:
            self.params = params
            self._rewind()

    def update_mask(self, mask: OverlayMask) -> None:
        if mask != self.mask:
            self.mask = mask
            self._rewind()

    def seek(self, target_index: int) -> bool:
        """处理到目标帧；返回是否成功读到该帧。"""
        if target_index < 0:
            return False
        if target_index < self.frame_index:
            self._rewind()
        while self.frame_index < target_index:
            ok, frame = self.capture.read()
            if not ok or frame is None or frame.size == 0:
                return False
            self.current_frame = frame
            self.current_result = self.pipeline.process(
                mask_recorded_overlays(frame, self.mask)
            )
            self.current_recorded_marker = detect_recorded_status_marker(frame)
            self.history_found.append(
                self.current_result.found or self.current_recorded_marker is True
            )
            self.frame_index += 1
        return self.current_frame is not None


def clip_roi(
    roi: tuple[int, int, int, int], frame_width: int, frame_height: int
) -> tuple[int, int, int, int]:
    """按 C++ 实现将 ROI 裁剪到图像边界。"""
    x, y, width, height = roi
    left = max(0, min(x, frame_width))
    top = max(0, min(y, frame_height))
    right = max(left, min(x + width, frame_width))
    bottom = max(top, min(y + height, frame_height))
    return left, top, right - left, bottom - top


def expanded_roi(
    blob: VisionBlob,
    range_x: int,
    range_y: int,
    frame_width: int,
    frame_height: int,
) -> tuple[int, int, int, int]:
    """以目标框为中心扩展下一帧 ROI。"""
    return clip_roi(
        (
            blob.x - range_x,
            blob.y - range_y,
            blob.width + 2 * range_x,
            blob.height + 2 * range_y,
        ),
        frame_width,
        frame_height,
    )


def load_vision_params(path: Path) -> VisionParams:
    """读取配置文件中的 guidance 视觉参数。"""
    parser = configparser.ConfigParser(interpolation=None)
    if not parser.read(path, encoding="utf-8"):
        raise ValueError(f"无法读取配置文件: {path}")
    if not parser.has_section("guidance"):
        raise ValueError(f"配置文件缺少 [guidance] 节: {path}")

    section = parser["guidance"]

    def get_int(name: str, default: int) -> int:
        try:
            return int(section.get(name, str(default)).strip())
        except ValueError as error:
            raise ValueError(f"配置项 {name} 不是整数") from error

    def get_float(name: str, default: float) -> float:
        try:
            return float(section.get(name, str(default)).strip())
        except ValueError as error:
            raise ValueError(f"配置项 {name} 不是浮点数") from error

    defaults = VisionParams()
    return VisionParams(
        initial_roi_x=get_int("initial_roi_x", defaults.initial_roi_x),
        initial_roi_y=get_int("initial_roi_y", defaults.initial_roi_y),
        initial_roi_width=get_int(
            "initial_roi_width", defaults.initial_roi_width
        ),
        initial_roi_height=get_int(
            "initial_roi_height", defaults.initial_roi_height
        ),
        h_min=get_int("h_min", defaults.h_min),
        h_max=get_int("h_max", defaults.h_max),
        s_min=get_int("s_min", defaults.s_min),
        s_max=get_int("s_max", defaults.s_max),
        v_min=get_int("v_min", defaults.v_min),
        v_max=get_int("v_max", defaults.v_max),
        found_range_x=get_int("found_range_x", defaults.found_range_x),
        found_range_y=get_int("found_range_y", defaults.found_range_y),
        min_blob_area=get_int("min_blob_area", defaults.min_blob_area),
        min_aspect_ratio=get_float(
            "min_aspect_ratio", defaults.min_aspect_ratio
        ),
        min_fill_ratio=get_float("min_fill_ratio", defaults.min_fill_ratio),
    )


class VisionReplayPipeline:
    """复刻 VisionRecognizer 的无 GUI 处理流程。"""

    def __init__(self, params: VisionParams):
        self.params = params
        self.current_roi = params.initial_roi()
        self.is_tracking = False

    def reset(self) -> None:
        """恢复到当前滑动条对应的初始 ROI。"""
        self.current_roi = self.params.initial_roi()
        self.is_tracking = False

    def update_params(self, params: VisionParams) -> None:
        """更新参数；初始 ROI 变化时同步重置当前 ROI。"""
        if params.initial_roi() != self.params.initial_roi():
            self.current_roi = params.initial_roi()
            self.is_tracking = False
        self.params = params

    @staticmethod
    def _is_valid_frame(frame: object) -> bool:
        return (
            isinstance(frame, np.ndarray)
            and frame.size > 0
            and frame.ndim == 3
            and frame.shape[2] == 3
            and frame.dtype == np.uint8
        )

    def _is_candidate(self, blob: VisionBlob) -> bool:
        if (
            blob.area < self.params.min_blob_area
            or blob.width <= 0
            or blob.height <= 0
        ):
            return False
        width = float(blob.width)
        height = float(blob.height)
        aspect_ratio = min(width / height, height / width)
        fill_ratio = blob.area / (width * height)
        return (
            aspect_ratio >= self.params.min_aspect_ratio
            and fill_ratio >= self.params.min_fill_ratio
        )

    def _scan_roi(
        self, frame: np.ndarray, roi: tuple[int, int, int, int]
    ) -> Optional[VisionBlob]:
        frame_height, frame_width = frame.shape[:2]
        x, y, width, height = clip_roi(roi, frame_width, frame_height)
        if width <= 0 or height <= 0:
            return None

        roi_image = frame[y : y + height, x : x + width].copy()
        hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
        h_min = int(np.clip(self.params.h_min, 0, 179))
        h_max = int(np.clip(self.params.h_max, 0, 179))
        s_min = int(np.clip(self.params.s_min, 0, 255))
        s_max = int(np.clip(self.params.s_max, 0, 255))
        v_min = int(np.clip(self.params.v_min, 0, 255))
        v_max = int(np.clip(self.params.v_max, 0, 255))
        mask = cv2.inRange(
            hsv,
            np.array((h_min, s_min, v_min), dtype=np.uint8),
            np.array((h_max, s_max, v_max), dtype=np.uint8),
        )
        if cv2.countNonZero(mask) == 0:
            return None

        label_count, labels, stats, centroids = (
            cv2.connectedComponentsWithStats(
                mask, connectivity=8, ltype=cv2.CV_32S
            )
        )
        del labels
        best_blob = VisionBlob()
        for label in range(1, int(label_count)):
            candidate = VisionBlob(
                x=int(stats[label, cv2.CC_STAT_LEFT]) + x,
                y=int(stats[label, cv2.CC_STAT_TOP]) + y,
                width=int(stats[label, cv2.CC_STAT_WIDTH]),
                height=int(stats[label, cv2.CC_STAT_HEIGHT]),
                area=int(stats[label, cv2.CC_STAT_AREA]),
                center_x=float(centroids[label, 0]) + x,
                center_y=float(centroids[label, 1]) + y,
            )
            if self._is_candidate(candidate):
                if candidate.area > best_blob.area:
                    best_blob = candidate

        return best_blob if best_blob.area > 0 else None

    def _scan_coarse_pooled(
        self, frame: np.ndarray
    ) -> Optional[tuple[int, int, int, int]]:
        frame_height, frame_width = frame.shape[:2]
        if frame_width < 2 or frame_height < 2:
            return None

        pooled_w = frame_width // 2
        pooled_h = frame_height // 2
        pooled_frame = cv2.resize(
            frame, (pooled_w, pooled_h), interpolation=cv2.INTER_AREA
        )
        pooled_hsv = cv2.cvtColor(pooled_frame, cv2.COLOR_BGR2HSV)
        h_min = int(np.clip(self.params.h_min, 0, 179))
        h_max = int(np.clip(self.params.h_max, 0, 179))
        s_min = int(np.clip(self.params.s_min, 0, 255))
        s_max = int(np.clip(self.params.s_max, 0, 255))
        v_min = int(np.clip(self.params.v_min, 0, 255))
        v_max = int(np.clip(self.params.v_max, 0, 255))
        mask = cv2.inRange(
            pooled_hsv,
            np.array((h_min, s_min, v_min), dtype=np.uint8),
            np.array((h_max, s_max, v_max), dtype=np.uint8),
        )
        if cv2.countNonZero(mask) == 0:
            return None

        label_count, labels, stats, centroids = (
            cv2.connectedComponentsWithStats(
                mask, connectivity=8, ltype=cv2.CV_32S
            )
        )
        del labels

        coarse_min_area = max(1, self.params.min_blob_area // 4)
        best_label = -1
        best_area = 0

        for label in range(1, int(label_count)):
            area = int(stats[label, cv2.CC_STAT_AREA])
            w = int(stats[label, cv2.CC_STAT_WIDTH])
            h = int(stats[label, cv2.CC_STAT_HEIGHT])
            if area < coarse_min_area or w <= 0 or h <= 0:
                continue

            aspect_ratio = min(w / h, h / w)
            fill_ratio = area / (w * h)
            if (
                aspect_ratio >= self.params.min_aspect_ratio * 0.7
                and fill_ratio >= self.params.min_fill_ratio * 0.7
            ):
                if area > best_area:
                    best_area = area
                    best_label = label

        if best_label < 0:
            return None

        c_x = int(stats[best_label, cv2.CC_STAT_LEFT])
        c_y = int(stats[best_label, cv2.CC_STAT_TOP])
        c_w = int(stats[best_label, cv2.CC_STAT_WIDTH])
        c_h = int(stats[best_label, cv2.CC_STAT_HEIGHT])

        full_x = c_x * 2
        full_y = c_y * 2
        full_w = c_w * 2
        full_h = c_h * 2
        margin_x = max(self.params.found_range_x, 2)
        margin_y = max(self.params.found_range_y, 2)

        coarse_roi = clip_roi(
            (
                full_x - margin_x,
                full_y - margin_y,
                full_w + 2 * margin_x,
                full_h + 2 * margin_y,
            ),
            frame_width,
            frame_height,
        )
        return coarse_roi if coarse_roi[2] > 0 and coarse_roi[3] > 0 else None

    def process(self, frame: np.ndarray) -> PipelineResult:
        """处理单帧并返回是否识别到目标及其位置。"""
        result = PipelineResult(next_roi=self.current_roi)
        if not self._is_valid_frame(frame):
            return result

        frame_height, frame_width = frame.shape[:2]

        try:
            detected_blob: Optional[VisionBlob] = None
            active_roi = self.current_roi

            if self.is_tracking:
                active_roi = clip_roi(self.current_roi, frame_width, frame_height)
                detected_blob = self._scan_roi(frame, active_roi)
            else:
                coarse_roi = self._scan_coarse_pooled(frame)
                if coarse_roi is not None:
                    active_roi = coarse_roi
                    detected_blob = self._scan_roi(frame, coarse_roi)

            result.roi = active_roi

            if detected_blob is not None and detected_blob.area > 0:
                result.found = True
                result.blob = detected_blob
                self.is_tracking = True
                self.current_roi = expanded_roi(
                    detected_blob,
                    self.params.found_range_x,
                    self.params.found_range_y,
                    frame_width,
                    frame_height,
                )
            else:
                result.found = False
                self.is_tracking = False
                self.current_roi = clip_roi(
                    self.params.initial_roi(), frame_width, frame_height
                )

            result.next_roi = self.current_roi
            return result
        except (cv2.error, ValueError):
            return result


def target_status_text(found: bool) -> str:
    """返回简洁的目标有无状态。"""
    return "Target: DETECTED" if found else "Target: NOT DETECTED"


def render_result_view(
    frame: np.ndarray,
    result: PipelineResult,
    status_override: Optional[str] = None,
) -> np.ndarray:
    """绘制普通识别画面、ROI、目标框和目标有无状态。"""
    view = frame.copy()
    x, y, width, height = result.next_roi
    if width > 0 and height > 0:
        cv2.rectangle(
            view,
            (x, y),
            (x + width - 1, y + height - 1),
            (0, 0, 255),
            2,
        )
    if result.found:
        cv2.rectangle(
            view,
            (result.blob.x, result.blob.y),
            (
                result.blob.x + result.blob.width - 1,
                result.blob.y + result.blob.height - 1,
            ),
            (255, 0, 0),
            2,
        )
        cv2.drawMarker(
            view,
            (round(result.blob.center_x), round(result.blob.center_y)),
            (0, 0, 255),
            cv2.MARKER_CROSS,
            12,
            2,
        )

    status_found = (
        status_override.startswith("Target: DETECTED")
        if status_override is not None else result.found
    )
    status_color = (0, 255, 0) if status_found else (0, 0, 255)
    cv2.putText(
        view,
        status_override or target_status_text(result.found),
        (6, max(18, view.shape[0] - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        status_color,
        1,
        cv2.LINE_AA,
    )
    return view


class TrackbarControls:
    """将 VisionParams 映射为 OpenCV 滑动条。"""

    WINDOW_NAME = "Vision Replay Controls"

    def __init__(
        self,
        params: VisionParams,
        frame_width: int,
        frame_height: int,
        mask: OverlayMask,
    ):
        self.frame_width = max(1, frame_width)
        self.frame_height = max(1, frame_height)
        self.max_range = max(self.frame_width, self.frame_height)
        self.max_area = max(1, self.frame_width * self.frame_height)
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WINDOW_NAME, 520, 780)
        self.specs = [
            ("ROI X", "initial_roi_x", max(0, self.frame_width - 1)),
            ("ROI Y", "initial_roi_y", max(0, self.frame_height - 1)),
            ("ROI W", "initial_roi_width", self.frame_width),
            ("ROI H", "initial_roi_height", self.frame_height),
            ("H min", "h_min", 179),
            ("H max", "h_max", 179),
            ("S min", "s_min", 255),
            ("S max", "s_max", 255),
            ("V min", "v_min", 255),
            ("V max", "v_max", 255),
            ("Range X", "found_range_x", self.max_range),
            ("Range Y", "found_range_y", self.max_range),
            ("Min area", "min_blob_area", self.max_area),
            ("Aspect x100", "min_aspect_ratio_x100", 100),
            ("Fill x100", "min_fill_ratio_x100", 100),
        ]
        for label, _, maximum in self.specs:
            value = self._initial_value(params, label)
            cv2.createTrackbar(
                label,
                self.WINDOW_NAME,
                max(0, min(value, maximum)),
                maximum,
                lambda _value: None,
            )
        self.mask_specs = [
            ("Mask W", mask.top_left_width, self.frame_width),
            ("Mask H", mask.top_left_height, self.frame_height),
            ("Bottom H", mask.bottom_height, self.frame_height),
            ("Ref X", mask.reference_x, max(0, self.frame_width - 1)),
            ("Ref Y", mask.reference_y, max(0, self.frame_height - 1)),
            ("Ref R", mask.reference_radius, self.max_range),
        ]
        for label, value, maximum in self.mask_specs:
            cv2.createTrackbar(
                label,
                self.WINDOW_NAME,
                max(0, min(value, maximum)),
                maximum,
                lambda _value: None,
            )

    @staticmethod
    def _initial_value(params: VisionParams, label: str) -> int:
        values = {
            "ROI X": params.initial_roi_x,
            "ROI Y": params.initial_roi_y,
            "ROI W": params.initial_roi_width,
            "ROI H": params.initial_roi_height,
            "H min": params.h_min,
            "H max": params.h_max,
            "S min": params.s_min,
            "S max": params.s_max,
            "V min": params.v_min,
            "V max": params.v_max,
            "Range X": params.found_range_x,
            "Range Y": params.found_range_y,
            "Min area": params.min_blob_area,
            "Aspect x100": round(params.min_aspect_ratio * 100),
            "Fill x100": round(params.min_fill_ratio * 100),
        }
        return int(values[label])

    def read(self) -> VisionParams:
        """读取当前滑动条值。"""
        values = {
            key: cv2.getTrackbarPos(label, self.WINDOW_NAME)
            for label, key, _ in self.specs
        }
        return VisionParams(
            initial_roi_x=values["initial_roi_x"],
            initial_roi_y=values["initial_roi_y"],
            initial_roi_width=values["initial_roi_width"],
            initial_roi_height=values["initial_roi_height"],
            h_min=values["h_min"],
            h_max=values["h_max"],
            s_min=values["s_min"],
            s_max=values["s_max"],
            v_min=values["v_min"],
            v_max=values["v_max"],
            found_range_x=values["found_range_x"],
            found_range_y=values["found_range_y"],
            min_blob_area=values["min_blob_area"],
            min_aspect_ratio=values["min_aspect_ratio_x100"] / 100.0,
            min_fill_ratio=values["min_fill_ratio_x100"] / 100.0,
        )

    def read_mask(self) -> OverlayMask:
        """读取录像旧可视化的遮挡范围。"""
        values = {
            label: cv2.getTrackbarPos(label, self.WINDOW_NAME)
            for label, _, _ in self.mask_specs
        }
        return OverlayMask(
            values["Mask W"],
            values["Mask H"],
            values["Bottom H"],
            values["Ref X"],
            values["Ref Y"],
            values["Ref R"],
        )

    def print_params(self) -> None:
        """打印可复制回配置文件的视觉参数。"""
        params = self.read()
        print("[vision replay] 当前视觉参数:")
        print(f"initial_roi_x={params.initial_roi_x}")
        print(f"initial_roi_y={params.initial_roi_y}")
        print(f"initial_roi_width={params.initial_roi_width}")
        print(f"initial_roi_height={params.initial_roi_height}")
        print(f"h_min={params.h_min}")
        print(f"h_max={params.h_max}")
        print(f"s_min={params.s_min}")
        print(f"s_max={params.s_max}")
        print(f"v_min={params.v_min}")
        print(f"v_max={params.v_max}")
        print(f"found_range_x={params.found_range_x}")
        print(f"found_range_y={params.found_range_y}")
        print(f"min_blob_area={params.min_blob_area}")
        print(f"min_aspect_ratio={params.min_aspect_ratio:.2f}")
        print(f"min_fill_ratio={params.min_fill_ratio:.2f}")


def build_argument_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        description="回放视频并交互调节视觉识别参数。"
    )
    parser.add_argument(
        "--video",
        help="待回放的视频文件路径；不指定时弹出 AVI 文件选择窗口",
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help=f"配置文件路径，默认 {DEFAULT_CONFIG_PATH}",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="视频结束后从第一帧循环播放",
    )
    parser.add_argument(
        "--raw-video",
        action="store_true",
        help="输入视频不含程序绘制的可视化内容，不应用默认遮挡",
    )
    return parser


def choose_video_file(initial_directory: Path) -> Optional[Path]:
    """未通过命令行指定视频时，从启动目录选择 AVI 文件。"""
    import tkinter as tk
    from tkinter import filedialog

    root: Optional[tk.Tk] = None
    try:
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        root.update()
        selected_path = filedialog.askopenfilename(
            parent=root,
            title="选择要回放的 AVI 视频",
            initialdir=str(initial_directory),
            filetypes=(("AVI 视频", "*.avi *.AVI"),),
        )
    except tk.TclError as error:
        raise OSError(f"无法打开 AVI 文件选择窗口：{error}") from error
    finally:
        if root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass

    return Path(selected_path) if selected_path else None


def sync_frame_trackbar(frame_index: int) -> bool:
    """同步进度条；窗口被关闭后停止当前录像回放。"""
    try:
        cv2.setTrackbarPos("Frame", "Vision Replay", frame_index)
    except cv2.error as error:
        if "trackbar" not in str(error).lower():
            raise
        return False
    return True


def run_replay(
    video_path: Path, config_path: Path, loop: bool, raw_video: bool = False
) -> int:
    """运行 GUI 回放主循环。"""
    params = load_vision_params(config_path)
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        print(f"错误：无法打开视频: {video_path}", file=sys.stderr)
        return 1

    ok, first_frame = capture.read()
    if not ok or first_frame is None or first_frame.size == 0:
        capture.release()
        print(f"错误：视频没有可读取的图像帧: {video_path}", file=sys.stderr)
        return 1

    frame_height, frame_width = first_frame.shape[:2]
    mask = load_overlay_mask(
        config_path, frame_width, frame_height, params.min_blob_area
    )
    if raw_video:
        mask = OverlayMask(0, 0, 0, mask.reference_x, mask.reference_y, 0)
    try:
        recorded_detections = load_recorded_detections(video_path)
    except (OSError, ValueError, TypeError) as error:
        print(f"警告：无法读取配套 CSV，继续只回放视频：{error}", file=sys.stderr)
        recorded_detections = {}
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    controls: Optional[TrackbarControls] = None
    try:
        controls = TrackbarControls(params, frame_width, frame_height, mask)
        cv2.namedWindow("Vision Replay", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Vision Replay", 1280, 720)
        if frame_count > 1:
            cv2.createTrackbar(
                "Frame", "Vision Replay", 0, frame_count - 1,
                lambda _value: None,
            )
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not np.isfinite(fps) or fps <= 0.0:
            fps = 30.0
        delay_ms = max(1, round(1000.0 / fps))
        replay = ReplaySession(capture, params, mask)
        current_params = params
        current_mask = mask
        desired_index = 0
        paused = False
        last_slider_index = 0
        print(
            "[vision replay] 空格暂停/继续，n 下一帧，b 上一帧，"
            "r 回到开头，拖动 Frame 跳转，p 打印参数，q 退出。"
        )

        while True:
            if frame_count > 1:
                slider_index = cv2.getTrackbarPos("Frame", "Vision Replay")
                if slider_index != last_slider_index:
                    desired_index = slider_index
                    paused = True
                last_slider_index = slider_index
            selected_params = controls.read()
            selected_mask = controls.read_mask()
            if selected_params != current_params:
                replay.update_params(selected_params)
                current_params = selected_params
            if selected_mask != current_mask:
                replay.update_mask(selected_mask)
                current_mask = selected_mask

            if not replay.seek(desired_index):
                if loop and desired_index > 0:
                    desired_index = 0
                    if not replay.seek(0):
                        print("错误：循环回放无法读取第一帧", file=sys.stderr)
                        return 1
                elif replay.current_frame is not None:
                    paused = True
                    desired_index = replay.frame_index
                else:
                    print("错误：无法读取回放帧", file=sys.stderr)
                    return 1

            assert replay.current_frame is not None
            assert replay.current_result is not None
            recognition_frame = mask_recorded_overlays(
                replay.current_frame, replay.mask
            )
            label_bar = np.zeros((45, frame_width * 2, 3), dtype=np.uint8)
            cv2.putText(
                label_bar, "RECORDED VIDEO", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
            )
            cv2.putText(
                label_bar, "MASKED RECOGNITION", (frame_width + 8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
            )
            recorded = recorded_detections.get(replay.frame_index)
            marker_found = replay.current_recorded_marker
            if marker_found is None and recorded is not None:
                marker_found = recorded.found
            counted_found = replay.current_result.found or marker_found is True
            if replay.frame_index < len(replay.history_found):
                replay.history_found[replay.frame_index] = counted_found
            baseline_text = (
                "Video marker: DETECTED" if marker_found is True else
                "Video marker: NOT DETECTED" if marker_found is False else
                "Video marker: N/A"
            )
            cv2.putText(
                label_bar, baseline_text, (8, 39),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA,
            )
            masked_target = recorded is not None and recorded_target_is_masked(
                recorded, replay.mask, frame_width, frame_height
            )
            if replay.current_result.found:
                review_text = "Current recognition: DETECTED"
                status_override = "Target: DETECTED"
            elif marker_found is True:
                review_text = (
                    "Counted by video marker; old box is masked"
                    if masked_target else "Counted by video marker"
                )
                status_override = "Target: DETECTED (RECORDED)"
            else:
                review_text = "Current recognition: NOT DETECTED"
                status_override = None
            cv2.putText(
                label_bar, review_text, (frame_width + 8, 39),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (0, 255, 0) if counted_found else (0, 0, 255), 1, cv2.LINE_AA,
            )
            result_view = render_result_view(
                recognition_frame, replay.current_result, status_override
            )
            timeline = np.zeros((9, frame_width * 2, 3), dtype=np.uint8)
            total_frames = max(frame_count, len(replay.history_found))
            for index, found in enumerate(replay.history_found):
                position = min(timeline.shape[1] - 1,
                               index * timeline.shape[1] // total_frames)
                timeline[:, position] = (0, 180, 0) if found else (0, 0, 180)
            marker = min(timeline.shape[1] - 1,
                         replay.frame_index * timeline.shape[1] // total_frames)
            timeline[:, marker] = (255, 255, 255)
            display = np.vstack(
                (label_bar, np.hstack((replay.current_frame, result_view)),
                 timeline)
            )
            cv2.imshow("Vision Replay", display)
            if frame_count > 1:
                last_slider_index = min(replay.frame_index, frame_count - 1)
                if not sync_frame_trackbar(last_slider_index):
                    break
            cv2.setWindowTitle(
                "Vision Replay",
                f"Vision Replay | frame={replay.frame_index} | "
                f"{target_status_text(replay.current_result.found)}",
            )
            key = cv2.waitKey(30 if paused else delay_ms) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                paused = not paused
            elif key == ord("n"):
                paused = True
                desired_index = replay.frame_index + 1
            elif key == ord("b"):
                paused = True
                desired_index = max(0, replay.frame_index - 1)
            elif key == ord("r"):
                paused = True
                desired_index = 0
            elif key == ord("p"):
                controls.print_params()

            if not paused and key not in (ord("n"), ord("b"), ord("r")):
                desired_index = replay.frame_index + 1
        return 0
    finally:
        capture.release()
        cv2.destroyAllWindows()


def main(argv: Optional[list[str]] = None) -> int:
    """命令行和桌面交互入口。"""
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        video_path = (
            Path(args.video)
            if args.video
            else choose_video_file(Path.cwd())
        )
        if video_path is None:
            return 0
        return run_replay(
            video_path, Path(args.config), args.loop, args.raw_video
        )
    except (OSError, ValueError, cv2.error) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
