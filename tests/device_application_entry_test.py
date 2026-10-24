#!/usr/bin/env python3
"""验证无参数设备入口、独立默认配置及失败启动后的安全清理。"""

import configparser
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

CALIBRATION_BINARY = Path(sys.argv.pop(1)).resolve()
GUIDANCE_BINARY = Path(sys.argv.pop(1)).resolve()
GUIDANCE_CONFIG = Path(sys.argv.pop(1)).resolve()
CALIBRATION_CONFIG = Path(sys.argv.pop(1)).resolve()
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


class DeviceApplicationEntryTest(unittest.TestCase):
    """使用真实配置加载流程，通过不存在的串口阻止硬件启动。"""

    def setUp(self):
        """创建隔离工作目录，并在每个用例结束后清理全部测试数据。"""
        temporary = tempfile.TemporaryDirectory(prefix="mosas device entry ")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.config_directory = self.root / "config"
        self.config_directory.mkdir()
        self.target = self.root / "capture data"

    def write_config(self, source, overrides=None):
        """复制默认配置并覆盖安全设备路径及采集参数。

        @param source 只读的原始默认配置文件。
        @param overrides 可选的 (section, key) 到值的映射；None 表示移除该键。
        @return 测试配置文件路径。
        """
        config = configparser.ConfigParser(interpolation=None)
        config.read_string(source.read_text())
        config["serial"]["device"] = str(self.root / "no-uart")
        if source == CALIBRATION_CONFIG:
            config["sync_calibration"]["output_dir"] = "../capture data"
            config["sync_calibration"]["seconds"] = "10"
            config["sync_calibration"]["fps"] = "20"
            config["sync_calibration"]["max_mib"] = "256"
        for (section, key), value in (overrides or {}).items():
            if value is None:
                config.remove_option(section, key)
            else:
                config[section][key] = value
        path = self.config_directory / source.name
        with path.open("w") as output:
            config.write(output)
        return path

    def run_app(self, binary):
        """不传递程序参数启动指定入口，返回已结束的子进程结果。"""
        return subprocess.run([str(binary)], cwd=self.root,
                              capture_output=True, text=True, timeout=5)

    def check_incomplete_capture(self):
        """核对采集设置、禁用控制输出标志及启动失败文件的完成状态。"""
        metadata = json.loads((self.target / "metadata.json").read_text())
        self.assertFalse(metadata["complete"])
        self.assertTrue(metadata["control_output_disabled"])
        self.assertEqual(metadata["end_reason"], "startup_failure")
        self.assertEqual(metadata["requested_seconds"], 10)
        self.assertEqual(metadata["recording_fps"], 20)
        self.assertEqual(metadata["max_data_bytes"], 256 * 1024 * 1024)
        self.assertEqual((self.target / "frames.gray8").stat().st_size, 0)
        self.assertFalse((self.target / "metadata.json.tmp").exists())
        self.assertFalse((self.config_directory / "capture data").exists())

    def test_missing_default_files(self):
        """各无参数入口只报告各自缺失的默认文件，不回退到另一份配置。"""
        for binary, config in [(GUIDANCE_BINARY, GUIDANCE_CONFIG),
                               (CALIBRATION_BINARY, CALIBRATION_CONFIG)]:
            with self.subTest(binary=binary):
                process = self.run_app(binary)
                self.assertEqual(process.returncode, 2, process.stderr)
                self.assertIn(f"config/{config.name}", process.stderr)
                self.assertIn("加载配置失败", process.stderr)

    def test_independent_defaults_and_failed_start_cleanup(self):
        """场上启动忽略标定配置，标定启动不要求场上配置存在。"""
        originals = {path: path.read_text() for path in [GUIDANCE_CONFIG, CALIBRATION_CONFIG]}
        field_path = self.write_config(GUIDANCE_CONFIG)
        calibration_path = self.config_directory / CALIBRATION_CONFIG.name
        calibration_path.write_text("invalid calibration configuration")
        field = self.run_app(GUIDANCE_BINARY)
        self.assertEqual(field.returncode, 1, field.stderr)
        self.assertIn("打开共享下位机串口", field.stderr)
        self.assertNotIn("[标定]", field.stdout)
        self.assertFalse(self.target.exists())

        field_path.unlink()
        self.write_config(CALIBRATION_CONFIG)
        capture = self.run_app(CALIBRATION_BINARY)
        self.assertEqual(capture.returncode, 1, capture.stderr)
        self.assertIn("打开共享下位机串口", capture.stderr)
        self.check_incomplete_capture()
        for path, original in originals.items():
            self.assertEqual(path.read_text(), original)

    def test_preflight_rejections(self):
        """在打开串口、启动 SDK 或创建输出目录之前拒绝无效配置。"""
        existing = self.root / "existing"
        existing.mkdir()
        cases = [
            {("sync_calibration", "output_dir"): None},
            {("sync_calibration", "output_dir"): ""},
            {("sync_calibration", "output_dir"): "../existing"},
            {("sync_calibration", "seconds"): "nan"},
            {("sync_calibration", "seconds"): "9"},
            {("sync_calibration", "seconds"): "601"},
            {("sync_calibration", "fps"): "30.5"},
            {("sync_calibration", "fps"): "0"},
            {("sync_calibration", "fps"): "61"},
            {("sync_calibration", "max_mib"): "-1"},
            {("sync_calibration", "max_mib"): "63"},
            {("sync_calibration", "max_mib"): "65537"},
            {("sync_calibration", "max_mib"): "64", ("sync_calibration", "seconds"): "600"},
            {("app", "enable_timing_statistics"): "maybe"},
            {("camera", "auto_exposure"): "true"},
            {("camera", "frame_timestamp_domain"): "unknown"},
            {("imu", "mode"): "simulated"},
        ]
        for overrides in cases:
            with self.subTest(overrides=overrides):
                self.write_config(CALIBRATION_CONFIG, overrides)
                process = self.run_app(CALIBRATION_BINARY)
                self.assertEqual(process.returncode, 2, process.stderr)
                self.assertNotIn("打开共享下位机串口", process.stderr)
                self.assertFalse(self.target.exists())

    def test_launchers(self):
        """源码脚本选择各自入口及项目根目录，并拒绝已移除的运行参数。"""
        scripts = self.root / "scripts"
        scripts.mkdir()
        unrelated = self.root / "unrelated working directory"
        unrelated.mkdir()
        shutil.copy2(SCRIPTS.parent / "CMakeLists.txt", self.root / "CMakeLists.txt")
        self.write_config(GUIDANCE_CONFIG)
        self.write_config(CALIBRATION_CONFIG)
        for name, binary in [("run_guidance.sh", GUIDANCE_BINARY),
                             ("run_sync_calibration.sh", CALIBRATION_BINARY)]:
            script = scripts / name
            shutil.copy2(SCRIPTS / name, script)
            for obsolete in [["--config", "unused"], ["--", "--timing"]]:
                with self.subTest(script=name, obsolete=obsolete):
                    process = subprocess.run([str(script), *obsolete], cwd=unrelated,
                                             capture_output=True, text=True, timeout=5)
                    self.assertEqual(process.returncode, 2, process.stderr)
            relative_build_directory = os.path.relpath(binary.parent, unrelated)
            process = subprocess.run([str(script), "--skip-build", "--build-dir", relative_build_directory],
                                     cwd=unrelated, capture_output=True, text=True, timeout=5)
            self.assertEqual(process.returncode, 1, process.stderr)
            self.assertIn("打开共享下位机串口", process.stderr)
            if binary == GUIDANCE_BINARY:
                self.assertFalse(self.target.exists())
                self.assertNotIn("[标定]", process.stdout)
            else:
                self.check_incomplete_capture()


if __name__ == "__main__":
    unittest.main()
