# 过期测试（保留备查，不参与构建）

这三个测试针对的是**本次重构之前**的接口，已经无法编译，因此移到这里保留备查，
而不是直接删除。它们没有被 git 跟踪（`.gitignore` 忽略 `/tests/`），删除即不可恢复，
所以采取移动的方式。

| 文件 | 引用的旧接口 | 现状 |
|---|---|---|
| `source_status_test.cpp` | `ImuSource`、`ImuStateSnapshot`、`SimulatedImuSource` | 接口已由 `PoseSource` / `PoseStateSnapshot` / `SimulatedPoseSource` 取代 |
| `serial_host_adapters_test.cpp` | `SerialImuSource`、`ImuSample`、`kImuFrameSize` | 20 字节加速度/角速度上行帧已被 14 字节姿态帧取代 |
| `serial_info_logging_test.cpp` | `SerialImuSource`、`ImuSample` | 同上 |

它们覆盖的行为并没有丢失，已由下列测试接管：

- 接收链路行为（帧同步、CRC 恢复、启动/运行积压、时间戳递增、诊断计数）
  → `tests/serial_pose_source_test.cpp`
- 数据源状态语义（ok / timeout / fatal / cancelled 与取消、停止）
  → `tests/pose_runtime_test.cpp`、`tests/source_status_test.cpp` 中的对应用例已并入
  `serial_pose_source_test.cpp` 的 `test_timeout_count_diagnostics` 与
  `test_cancel_and_fatal_results`
- 指令输出与日志（`SerialCommandSink` / `LoggingCommandSink`）
  → `tests/app_config_pose_test.cpp`（构造与配置）与
  `tests/command_dispatch_test.cpp`（投递、失败与停止语义）

如果需要让它们重新可用，请基于当前的 `PoseSource` / `SerialPoseSource` 接口改写，
而不是把它们移回 `tests/` 根目录。
