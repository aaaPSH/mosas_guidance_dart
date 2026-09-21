#!/usr/bin/env python3
"""串口帧调试程序的伪终端端到端测试。"""

import os
import pty
import select
import signal
import struct
import subprocess
import sys
import time


def calculate_crc(data: bytes) -> int:
    """计算协议使用的 CRC-16/CCITT-FALSE。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def make_imu_frame() -> bytes:
    """构造一帧有效的下位机 IMU 数据。"""
    payload = struct.pack("<hhhhhhH", 100, -200, 300, 10, -20, 30, 0x1234)
    return b"\x55\xAA" + payload + struct.pack("<H", calculate_crc(payload)) + b"\x0D\x0A"


def make_control_frame() -> bytes:
    """构造三个字段均为 1.0 的上行控制帧。"""
    payload = struct.pack("<hhh", 100, 100, 100) + b"\x00\x00"
    return b"\xAA\x55" + payload + struct.pack("<H", calculate_crc(payload)) + b"\x0D\x0A"


def read_until(master_fd: int, expected: bytes, timeout: float) -> bytes:
    """从伪终端读取到指定字节串。"""
    deadline = time.monotonic() + timeout
    received = bytearray()
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        readable, _, _ = select.select([master_fd], [], [], remaining)
        if not readable:
            continue
        received.extend(os.read(master_fd, 4096))
        if expected in received:
            return bytes(received)
    raise AssertionError(
        f"在 {timeout:.1f} 秒内没有收到 {expected!r}，实际收到 {bytes(received)!r}"
    )


def read_until_count(
    master_fd: int, expected: bytes, count: int, timeout: float
) -> bytes:
    """从伪终端读取到指定字节串出现足够次数。"""
    deadline = time.monotonic() + timeout
    received = bytearray()
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        readable, _, _ = select.select([master_fd], [], [], remaining)
        if not readable:
            continue
        received.extend(os.read(master_fd, 4096))
        if bytes(received).count(expected) >= count:
            return bytes(received)
    raise AssertionError(
        f"在 {timeout:.1f} 秒内没有收到 {expected!r} 至少 {count} 次，"
        f"实际收到 {bytes(received)!r}"
    )


def read_output_until(stream, expected: str, timeout: float) -> str:
    """读取程序输出到指定文本。"""
    deadline = time.monotonic() + timeout
    received = ""
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        readable, _, _ = select.select([stream], [], [], remaining)
        if not readable:
            continue
        chunk = os.read(stream.fileno(), 4096).decode("utf-8", errors="replace")
        received += chunk
        if expected in received:
            return received
    raise AssertionError(
        f"在 {timeout:.1f} 秒内没有看到 {expected!r}，实际输出为 {received!r}"
    )


def main() -> int:
    if len(sys.argv) != 2:
        raise AssertionError("用法：serial_frame_debug_app_test.py APP_PATH")

    app_path = sys.argv[1]
    if not os.access(app_path, os.X_OK):
        raise AssertionError(f"待测试的串口调试程序不存在或不可执行：{app_path}")

    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    process = subprocess.Popen(
        [
            app_path,
            "--device",
            slave_path,
            "--read-timeout-ms",
            "20",
            "--write-timeout-ms",
            "50",
            "--send-interval-ms",
            "5",
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    output = ""
    try:
        output += read_output_until(process.stdout, "已打开", 2.0)
        os.write(master_fd, make_imu_frame())
        output += read_output_until(process.stdout, "[接收]", 2.0)
        if "55 AA" not in output:
            raise AssertionError(f"接收日志没有打印原始帧：{output!r}")

        overlap = b"\x55\xAA" + b"\x01" * 8 + make_imu_frame()[:10]
        os.write(master_fd, overlap)
        output += read_output_until(process.stdout, "校验失败", 2.0)
        os.write(master_fd, make_imu_frame()[10:])
        output += read_output_until(
            process.stdout,
            "bytes=55 AA 64 00 38 FF 2C 01 0A 00 EC FF 1E 00 34 12 6B 25 0D 0A",
            2.0,
        )
        if "acceleration_mps2=(1.000,-2.000,3.000)" not in output:
            raise AssertionError(f"接收日志没有打印解码后的加速度：{output!r}")
        if "angular_velocity_rad_s=(1.000,-2.000,3.000)" not in output:
            raise AssertionError(f"接收日志没有打印解码后的角速度：{output!r}")

        process.stdin.write(b"x")
        process.stdin.flush()
        output += read_output_until(process.stdout, "开始发送", 2.0)

        received = read_until_count(master_fd, make_control_frame(), 2, 2.0)
        if received.count(make_control_frame()) < 2:
            raise AssertionError(f"没有连续收到两帧控制数据：{received!r}")
        if "AA 55" in output:
            raise AssertionError(f"发送日志不应打印控制帧字节：{output!r}")
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
        if process.stdout is not None:
            output += process.stdout.read().decode("utf-8", errors="replace")
        os.close(master_fd)
        os.close(slave_fd)

    if process.returncode != 0:
        raise AssertionError(f"程序退出码为 {process.returncode}，输出为 {output!r}")
    print("serial_frame_debug_app_test passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
