#!/usr/bin/env python3
"""录制视频视觉流程回放与参数调节工具。"""

from __future__ import annotations

import argparse
import configparser
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

    def reset(self) -> None:
        """恢复到当前滑动条对应的初始 ROI。"""
        self.current_roi = self.params.initial_roi()

    def update_params(self, params: VisionParams) -> None:
        """更新参数；初始 ROI 变化时同步重置当前 ROI。"""
        if params.initial_roi() != self.params.initial_roi():
            self.current_roi = params.initial_roi()
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

    def process(self, frame: np.ndarray) -> PipelineResult:
        """处理单帧并返回是否识别到目标及其位置。"""
        result = PipelineResult(next_roi=self.current_roi)
        if not self._is_valid_frame(frame):
            return result

        frame_height, frame_width = frame.shape[:2]
        roi = clip_roi(self.current_roi, frame_width, frame_height)
        result.roi = roi
        result.next_roi = roi
        x, y, width, height = roi
        if width <= 0 or height <= 0:
            self.current_roi = clip_roi(
                self.params.initial_roi(), frame_width, frame_height
            )
            result.next_roi = self.current_roi
            return result

        try:
            roi_image = frame[y : y + height, x : x + width].copy()
            hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
            h_min, h_max = sorted(
                (int(np.clip(self.params.h_min, 0, 179)),
                 int(np.clip(self.params.h_max, 0, 179)))
            )
            s_min, s_max = sorted(
                (int(np.clip(self.params.s_min, 0, 255)),
                 int(np.clip(self.params.s_max, 0, 255)))
            )
            v_min, v_max = sorted(
                (int(np.clip(self.params.v_min, 0, 255)),
                 int(np.clip(self.params.v_max, 0, 255)))
            )
            mask = cv2.inRange(
                hsv,
                np.array((h_min, s_min, v_min), dtype=np.uint8),
                np.array((h_max, s_max, v_max), dtype=np.uint8),
            )
            if cv2.countNonZero(mask) == 0:
                self.current_roi = clip_roi(
                    self.params.initial_roi(), frame_width, frame_height
                )
                result.next_roi = self.current_roi
                return result

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

            if best_blob.area > 0:
                result.found = True
                result.blob = best_blob
                self.current_roi = expanded_roi(
                    best_blob,
                    self.params.found_range_x,
                    self.params.found_range_y,
                    frame_width,
                    frame_height,
                )
            else:
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


def render_result_view(frame: np.ndarray, result: PipelineResult) -> np.ndarray:
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

    status_color = (0, 255, 0) if result.found else (0, 0, 255)
    cv2.putText(
        view,
        target_status_text(result.found),
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

    def __init__(self, params: VisionParams, frame_width: int, frame_height: int):
        self.frame_width = max(1, frame_width)
        self.frame_height = max(1, frame_height)
        self.max_range = max(self.frame_width, self.frame_height)
        self.max_area = max(1, self.frame_width * self.frame_height)
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WINDOW_NAME, 520, 520)
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
    parser.add_argument("--video", required=True, help="待回放的视频文件路径")
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
    return parser


def run_replay(video_path: Path, config_path: Path, loop: bool) -> int:
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
    pipeline = VisionReplayPipeline(params)
    current_params = params
    controls: Optional[TrackbarControls] = None
    try:
        controls = TrackbarControls(params, frame_width, frame_height)
        cv2.namedWindow("Vision Replay", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Vision Replay", 1280, 960)
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not np.isfinite(fps) or fps <= 0.0:
            fps = 30.0
        delay_ms = max(1, round(1000.0 / fps))

        current_frame: Optional[np.ndarray] = first_frame
        current_result: Optional[PipelineResult] = None
        frame_index = -1
        paused = False
        read_next = True

        while True:
            selected_params = controls.read()
            if selected_params != current_params:
                pipeline.update_params(selected_params)
                current_params = selected_params
                if current_frame is not None:
                    current_result = pipeline.process(current_frame)

            if read_next:
                if frame_index >= 0 or current_frame is not first_frame:
                    ok, current_frame = capture.read()
                if not ok or current_frame is None:
                    if not loop:
                        break
                    capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    pipeline.reset()
                    frame_index = -1
                    ok, current_frame = capture.read()
                    if not ok or current_frame is None:
                        print("错误：循环回放无法读取第一帧", file=sys.stderr)
                        return 1
                frame_index += 1
                current_result = pipeline.process(current_frame)
                read_next = False

            if current_frame is None or current_result is None:
                continue
            result_view = render_result_view(current_frame, current_result)
            cv2.imshow("Vision Replay", result_view)
            cv2.setWindowTitle(
                "Vision Replay",
                f"Vision Replay | frame={frame_index} | "
                f"{target_status_text(current_result.found)}",
            )
            key = cv2.waitKey(30 if paused else delay_ms) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                paused = not paused
            elif key == ord("n") and paused:
                read_next = True
            elif key == ord("r"):
                pipeline.reset()
                current_result = pipeline.process(current_frame)
            elif key == ord("p"):
                controls.print_params()

            if not paused:
                read_next = True
        return 0
    finally:
        capture.release()
        cv2.destroyAllWindows()


def main(argv: Optional[list[str]] = None) -> int:
    """命令行入口。"""
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        return run_replay(Path(args.video), Path(args.config), args.loop)
    except (OSError, ValueError, cv2.error) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
