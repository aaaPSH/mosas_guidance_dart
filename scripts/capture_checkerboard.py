#!/usr/bin/env python3
"""采集相机标定用的棋盘格图片，支持窗口预览和无屏幕模式。"""

from __future__ import annotations

import argparse
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence, Tuple, Union

try:
    import cv2
except ImportError:
    cv2 = None


def camera_source(value: str) -> Union[int, str]:
    """将纯数字解释为摄像头编号，其余内容作为设备路径。"""
    return int(value) if value.isdecimal() else value


def positive_integer(value: str) -> int:
    """解析大于零的命令行整数。"""
    try:
        number = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("请输入整数") from error
    if number <= 0:
        raise argparse.ArgumentTypeError("数值必须大于零")
    return number


def positive_float(value: str) -> float:
    """解析有限且大于零的命令行浮点数。"""
    try:
        number = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("请输入数字") from error
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("数值必须是有限正数")
    return number


def corner_dimension(value: str) -> int:
    """解析至少包含三个角点的棋盘格尺寸。"""
    number = positive_integer(value)
    if number < 3:
        raise argparse.ArgumentTypeError("内角点行列数必须至少为 3")
    return number


def parse_arguments(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析采集脚本的命令行参数。"""
    parser = argparse.ArgumentParser(
        description="采集摄像头画面中的棋盘格标定图片，支持窗口预览或无屏幕自动采集。"
    )
    parser.add_argument(
        "--camera",
        type=camera_source,
        default=0,
        help="摄像头编号或设备路径，默认 0",
    )
    parser.add_argument(
        "--columns",
        type=corner_dimension,
        required=True,
        help="棋盘格内角点列数",
    )
    parser.add_argument(
        "--rows",
        type=corner_dimension,
        required=True,
        help="棋盘格内角点行数",
    )
    parser.add_argument(
        "--width",
        type=positive_integer,
        default=640,
        help="请求的采集宽度，默认 640",
    )
    parser.add_argument(
        "--height",
        type=positive_integer,
        default=480,
        help="请求的采集高度，默认 480",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("calibration_images"),
        help="图片会话目录的根目录，默认 calibration_images",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="无屏幕模式：检测到棋盘格后自动保存",
    )
    parser.add_argument(
        "--target-count",
        type=positive_integer,
        default=20,
        help="无屏幕模式的目标图片数，默认 20",
    )
    parser.add_argument(
        "--save-interval",
        type=positive_float,
        default=2.0,
        help="无屏幕模式两次保存的最小间隔秒数，默认 2",
    )
    return parser.parse_args(argv)


def create_session_directory(output_root: Path) -> Path:
    """创建本地时间命名的采集目录，遇到重名时追加序号。"""
    output_root.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    candidate = output_root / timestamp
    suffix = 1
    while True:
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            candidate = output_root / f"{timestamp}_{suffix:02d}"
            suffix += 1


def detect_checkerboard(
    frame, pattern_size: Tuple[int, int]
) -> Tuple[bool, Optional[object]]:
    """检测并细化棋盘格内角点。"""
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("摄像头必须输出三通道 BGR 图像")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray,
        pattern_size,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found or corners is None:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    refined_corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1), criteria
    )
    return True, refined_corners


def run_headless_capture(
    args: argparse.Namespace,
    capture,
    frame,
    session_dir: Path,
) -> int:
    """在没有图形界面的环境中按间隔自动采集棋盘格图片。"""
    saved_count = 0
    next_save_time = 0.0
    last_status_time = 0.0
    print(
        f"无屏幕采集：目标 {args.target_count} 张，"
        f"保存间隔至少 {args.save_interval:g} 秒。"
    )
    print("请在镜头前持续改变棋盘格的位置、距离和倾角；按 Ctrl+C 可提前结束。")

    while saved_count < args.target_count:
        if frame.ndim != 3 or frame.shape[2] != 3:
            print("摄像头必须输出三通道 BGR 图像。", file=sys.stderr)
            return 1

        # 与主程序保持相同的 180 度相机帧方向。
        oriented_frame = cv2.flip(frame, -1)
        found, _ = detect_checkerboard(
            oriented_frame, (args.columns, args.rows)
        )
        current_time = time.monotonic()

        if found and current_time >= next_save_time:
            image_path = session_dir / f"frame_{saved_count + 1:04d}.png"
            save_error = None
            try:
                saved = cv2.imwrite(str(image_path), oriented_frame)
            except cv2.error as error:
                saved = False
                save_error = str(error)

            if not saved:
                detail = f"：{save_error}" if save_error else ""
                print(f"保存图片失败 {image_path}{detail}", file=sys.stderr)
                return 1

            saved_count += 1
            next_save_time = current_time + args.save_interval
            last_status_time = current_time
            print(
                f"已保存 {saved_count}/{args.target_count}：{image_path}；"
                "请改变棋盘姿态后再继续采集。"
            )
            if saved_count >= args.target_count:
                print("已达到目标张数，采集完成。")
                return 0
        elif current_time - last_status_time >= 5.0:
            if found:
                print("已检测到棋盘格，正在等待下一保存间隔；请改变棋盘姿态。")
            else:
                print("暂未检测到完整棋盘格，请将棋盘移入镜头并调整角度。")
            last_status_time = current_time

        ok, frame = capture.read()
        if not ok or frame is None or frame.size == 0:
            print("摄像头读取图像失败。", file=sys.stderr)
            return 1

    return 0


def run_capture(args: argparse.Namespace) -> int:
    """运行摄像头预览采集或无屏幕自动采集流程。"""
    if cv2 is None:
        package = "opencv-python-headless" if args.headless else "opencv-python"
        print(
            f"缺少 OpenCV Python 依赖，请运行 python3 -m pip install {package}。",
            file=sys.stderr,
        )
        return 1

    capture = None
    window_created = False
    try:
        capture = cv2.VideoCapture(args.camera)
        if not capture.isOpened():
            print(f"无法打开摄像头：{args.camera}", file=sys.stderr)
            return 1

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

        ok, frame = capture.read()
        if not ok or frame is None or frame.size == 0:
            print(f"无法从摄像头读取图像：{args.camera}", file=sys.stderr)
            return 1

        try:
            session_dir = create_session_directory(args.output_dir)
        except OSError as error:
            print(f"无法创建图片目录 {args.output_dir}：{error}", file=sys.stderr)
            return 1

        height, width = frame.shape[:2]
        print(f"摄像头：{args.camera}，实际分辨率：{width}x{height}")
        print(f"图片目录：{session_dir}")
        if args.headless:
            return run_headless_capture(args, capture, frame, session_dir)

        print("移动棋盘格以改变位置、大小和倾角；按空格保存，按 q 或 Esc 退出。")

        window_name = "Checkerboard Capture"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        window_created = True
        pattern_size = (args.columns, args.rows)
        saved_count = 0

        while True:
            if frame.ndim != 3 or frame.shape[2] != 3:
                print("摄像头必须输出三通道 BGR 图像。", file=sys.stderr)
                return 1

            # 与主程序保持相同的 180 度相机帧方向。
            oriented_frame = cv2.flip(frame, -1)
            found, corners = detect_checkerboard(oriented_frame, pattern_size)

            preview = oriented_frame.copy()
            if found:
                cv2.drawChessboardCorners(preview, pattern_size, corners, True)

            status = "BOARD READY - SPACE TO SAVE" if found else "BOARD NOT FOUND"
            status_color = (0, 220, 0) if found else (0, 0, 255)
            cv2.putText(
                preview,
                status,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                status_color,
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                preview,
                f"Resolution: {width}x{height}  Saved: {saved_count}",
                (15, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                preview,
                "Move the board for a new view | Q/ESC to quit",
                (15, max(24, height - 16)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow(window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                return 0

            if key == ord(" "):
                if not found:
                    print("未检测到完整棋盘格，请调整后重试。")
                else:
                    image_path = session_dir / f"frame_{saved_count + 1:04d}.png"
                    save_error = None
                    try:
                        saved = cv2.imwrite(str(image_path), oriented_frame)
                    except cv2.error as error:
                        saved = False
                        save_error = str(error)
                    if saved:
                        saved_count += 1
                        print(f"已保存：{image_path}")
                    else:
                        detail = f"：{save_error}" if save_error else ""
                        print(f"保存图片失败 {image_path}{detail}", file=sys.stderr)

            ok, frame = capture.read()
            if not ok or frame is None or frame.size == 0:
                print(f"摄像头读取图像失败：{args.camera}", file=sys.stderr)
                return 1
            height, width = frame.shape[:2]
    except OSError as error:
        print(f"相机采集或文件操作失败：{error}", file=sys.stderr)
        return 1
    except ValueError as error:
        print(f"图像格式无效：{error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n采集已中断。")
        return 130
    except cv2.error as error:
        action = "图像处理" if args.headless else "预览或图像处理"
        print(
            f"OpenCV {action}失败：{error}",
            file=sys.stderr,
        )
        return 1
    finally:
        if capture is not None:
            capture.release()
        if window_created:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


def main(argv: Optional[Sequence[str]] = None) -> int:
    """解析参数并启动采集。"""
    args = parse_arguments(argv)
    return run_capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
