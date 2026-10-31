#!/usr/bin/env python3
"""验证安装产物的无参数启动、配置定位与源码配置隔离，不访问真实设备。"""

import configparser
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

BUILD_DIRECTORY = Path(sys.argv.pop(1)).resolve()


class DeviceApplicationInstallTest(unittest.TestCase):
    """使用真实 CMake 安装产物，所有运行配置均指向不存在的串口。"""

    @classmethod
    def setUpClass(cls):
        """安装到临时前缀，测试结束时恢复构建目录的安装清单并清理产物。"""
        temporary = tempfile.TemporaryDirectory(prefix="mosas installed runtime ")
        cls.addClassCleanup(temporary.cleanup)
        cls.root = Path(temporary.name)
        cls.prefix = cls.root / "installation"
        cls.config_directory = cls.prefix / "etc/mosas"
        cls.unrelated = cls.root / "unrelated working directory"
        cls.unrelated.mkdir()
        manifest = BUILD_DIRECTORY / "install_manifest.txt"
        original_manifest = manifest.read_bytes() if manifest.exists() else None

        def restore_manifest():
            if original_manifest is None:
                manifest.unlink(missing_ok=True)
            else:
                manifest.write_bytes(original_manifest)

        cls.addClassCleanup(restore_manifest)
        process = subprocess.run(["cmake", "--install", str(BUILD_DIRECTORY), "--prefix", str(cls.prefix)],
                                 capture_output=True, text=True, timeout=30)
        if process.returncode:
            raise RuntimeError(process.stdout + process.stderr)
        cls.defaults = {name: (cls.config_directory / name).read_text(encoding="utf-8")
                        for name in ["mosas_guidance.conf", "mosas_sync_calibration.conf"]}

    def setUp(self):
        """每个用例使用独立的采集目录，防止已有目录影响启动预检。"""
        self.capture_number = 0
        self.config_directory.mkdir(parents=True, exist_ok=True)

    def write_safe_config(self, name, directory=None):
        """写入安全配置，返回标定输出目录；相对目录应按安装配置的位置解析。"""
        config = configparser.ConfigParser(interpolation=None)
        config.read_string(self.defaults[name])
        config["serial"]["device"] = str(self.root / "no-uart")
        capture = None
        if name == "mosas_sync_calibration.conf":
            self.capture_number += 1
            capture = self.prefix / f"capture {self._testMethodName} {self.capture_number}"
            config["sync_calibration"]["output_dir"] = f"../../{capture.name}"
            config["sync_calibration"]["seconds"] = "10"
            config["sync_calibration"]["fps"] = "20"
            config["sync_calibration"]["max_mib"] = "256"
        target_directory = directory or self.config_directory
        target_directory.mkdir(parents=True, exist_ok=True)
        with (target_directory / name).open("w", encoding="utf-8") as output:
            config.write(output)
        return capture

    def test_installed_entries_and_launchers(self):
        """安装程序和脚本从任意工作目录启动，只读取安装配置，不回退到当前目录。"""
        decoy = self.unrelated / "config"
        decoy.mkdir(exist_ok=True)
        for name, binary, launcher in [
            ("mosas_guidance.conf", "mosas_guidance_app", "run_guidance.sh"),
            ("mosas_sync_calibration.conf", "mosas_sync_calibration_app", "run_sync_calibration.sh"),
        ]:
            (decoy / name).write_text("invalid working-directory configuration", encoding="utf-8")
            for executable in [binary, launcher]:
                with self.subTest(executable=executable):
                    capture = self.write_safe_config(name)
                    process = subprocess.run([str(self.prefix / "bin" / executable)], cwd=self.unrelated,
                                             capture_output=True, text=True, timeout=5)
                    self.assertEqual(process.returncode, 1, process.stdout + process.stderr)
                    self.assertIn("打开共享下位机串口", process.stderr)
                    self.assertNotIn("CMake Error", process.stderr)
                    if capture is not None:
                        metadata = json.loads((capture / "metadata.json").read_text(encoding="utf-8"))
                        self.assertFalse(metadata["complete"])
                        self.assertTrue(metadata["control_output_disabled"])
                        self.assertEqual(metadata["end_reason"], "startup_failure")

    def test_missing_installed_configuration_does_not_fall_back(self):
        """安装配置目录缺失时应明确报错，不采用工作目录中另一份有效配置。"""
        decoy = self.unrelated / "config"
        for name in self.defaults:
            self.write_safe_config(name, decoy)
        shutil.rmtree(self.config_directory)
        for name, binary in [("mosas_guidance.conf", "mosas_guidance_app"),
                             ("mosas_sync_calibration.conf", "mosas_sync_calibration_app")]:
            with self.subTest(binary=binary):
                process = subprocess.run([str(self.prefix / "bin" / binary)], cwd=self.unrelated,
                                         capture_output=True, text=True, timeout=5)
                self.assertEqual(process.returncode, 2, process.stderr)
                self.assertIn(str(self.config_directory / name), process.stderr)
                self.assertNotIn("打开共享下位机串口", process.stderr)


if __name__ == "__main__":
    unittest.main()
