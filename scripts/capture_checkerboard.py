#!/usr/bin/env python3
"""实时预览并采集相机标定用的棋盘格图片。"""

from __future__ import annotations

import argparse
import sys
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


def corner_dimension(value: str) -> int:
    """解析至少包含三个角点的棋盘格尺寸。"""
    number = positive_integer(value)
    if number < 3:
        raise argparse.ArgumentTypeError("内角点行列数必须至少为 3")
    return number


def parse_arguments(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析采集脚本的命令行参数。"""
    parser = argparse.ArgumentParser(
        description="预览摄像头画面，并保存检测到棋盘格的标定图片。"
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


def run_capture(args: argparse.Namespace) -> int:
    """运行摄像头预览和键盘采集流程。"""
    if cv2 is None:
        print(
            "缺少 OpenCV Python 依赖，请安装带图形界面的 opencv-python。",
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
    except cv2.error as error:
        print(
            f"OpenCV 预览或图像处理失败，请确认安装了带图形界面的 opencv-python：{error}",
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
