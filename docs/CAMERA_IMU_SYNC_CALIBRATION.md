# 在原程序负载下标定相机／IMU 相对时延

## 原理和限制

标定采集与实际场上运行使用独立主函数和启动脚本：

| 用途 | 主函数 | 默认配置文件 | 启动脚本 |
|---|---|---|---|
| 场上运行 | `main/mosas_guidance_app.cpp` → `mosas_guidance_app` | `config/mosas_guidance.conf` | `scripts/run_guidance.sh` |
| 标定采集 | `main/mosas_sync_calibration_app.cpp` → `mosas_sync_calibration_app` | `config/mosas_sync_calibration.conf` | `scripts/run_sync_calibration.sh` |

两个入口均为无参数 `main()`，分别读取各自的默认配置文件，不再解析任何命令行参数。表中路径对应源码树；安装后读取 CMake 配置的安装目录中的同名文件，默认位于 `<prefix>/etc/mosas/`，不采用当前工作目录的配置。两个配置文件各自包含完整的设备、算法和输出设置，互不引用、互不覆盖；修改其中一个不会改变另一个。标定入口始终禁止控制输出，两者复用设备初始化、算法、输出和统计逻辑。

标定时相机和串口各打开一次，原有采集、预处理、姿态、视觉、计算及配置中的录像／RTSP 路径继续运行；旁路记录数据，结束后使用独立 Python 脚本离线检测棋盘格并匹配视觉旋转与陀螺仪积分。

**不添加人工 CPU、内存或磁盘压力，不启动第二个设备采集程序。** 图像拷贝和记录仍有额外开销，会单独统计。

本项目相机倒装 180°，现有内参来自翻转后的正立图像：

```text
原 SDK 图像 ── 原 prepare：resize（如有）＋翻转 180° ── 正常处理
    └─ 独立拷贝、后台转灰度并保存
              └─ 离线 resize（如有）＋翻转 180° ── 使用现有正立内参
```

两条图像路径各翻转一次，不能再手工翻转数据文件或把内参改回原 SDK 方向。离线几何变换不改变图像时间戳。

这得到的是**本次台架场景下的相对固定偏移**，不是绝对硬件同步，也不是“图像到计算结果”的处理延迟。IMU 协议没有 MCU 采样时间或样本号，主机接收估计仍可能受调度影响。

## 1. 安全与采集条件

- **执行器必须物理断能。** 标定模式使用不持有串口的内存命令接收端，不构造串口命令发送器，连清零帧也不发送。停发不保证下位机清除旧指令。
- 相机与 IMU 必须刚性固定在同一物体上，棋盘格固定不动；转动相机／IMU 整体，不要移动棋盘格。
- 在 `config/mosas_sync_calibration.conf` 中独立核对正常部署的串口波特率、接收优先级、相机帧率、曝光、输出模式和时间补偿；场上配置的变更不会自动同步过来。要求 `imu.mode=serial`、相机 `mode=free_run`、`auto_exposure=false`，且 SDK 时间域已明确验证。
- 静止／手动转动不会执行所有发射后计算分支，match 模式也不一定进入触发录像窗口。不要人为伪造起飞状态；通过报告的实际阶段样本数判断覆盖范围。
- 关闭命令输出也移除了串口 TX 线程及发送 I/O 负载，因此不能声称与完整双向串口运行完全等价。

## 2. 构建

在已有应用构建目录中执行：

```sh
cmake -S . -B build/guidance-app
cmake --build build/guidance-app --target mosas_guidance_app mosas_sync_calibration_app -j2
```

离线分析需要 Python 3、NumPy 和 OpenCV Python 模块。确认环境已具备依赖，不需要在目标板采集时运行 Python 或棋盘格检测。

需要安装部署时执行，例如：

```sh
cmake --install build/guidance-app --prefix /opt/mosas
```

安装后的两个启动脚本位于 `/opt/mosas/bin/`，直接运行即可，不再构建源码。分别修改 `/opt/mosas/etc/mosas/mosas_guidance.conf` 和 `/opt/mosas/etc/mosas/mosas_sync_calibration.conf`；它们仍相互独立。Linux 程序通过自身可执行文件位置定位配置，可从其他工作目录启动。缺失安装配置时会报错，不回退到当前目录。

安装配置中的相对记录路径仍按配置文件所在目录解析；部署前须重新核对 `output.recording_path`、`sync_calibration.output_dir` 及目录写入权限，不能把源码树中的相对目录直接当成安装根目录。

## 3. 采集

先修改 `config/mosas_sync_calibration.conf`：

```ini
[app]
enable_timing_statistics=true

[sync_calibration]
output_dir=../data/sync_capture_01
seconds=45
fps=30
max_mib=2048
```

以上为文件中对应配置节，不要重复追加同名键。`output_dir` 必须是不存在的新目录，拒绝覆盖；相对路径按配置文件所在目录解析，示例最终写入项目的 `data/sync_capture_01`。每次采集前修改为新的目录，程序不会改写配置。

```sh
scripts/run_sync_calibration.sh --skip-build
```

省略 `--skip-build` 时脚本会自动配置并构建标定程序。脚本仅接受构建选项，会切换到项目根目录并无参数启动程序。也可在项目根目录直接调用：

```sh
build/guidance-app/mosas_sync_calibration_app
```

实际场上运行使用 `scripts/run_guidance.sh`，读取独立的 `config/mosas_guidance.conf`；其耗时统计也由该文件的 `[app] enable_timing_statistics` 决定。

| 配置项 | 含义 |
|---|---|
| `sync_calibration.output_dir` | 必填，新的标定采集目录，相对配置文件所在目录解析 |
| `sync_calibration.seconds` | 10～600 秒，默认 45 秒；太短可能无法通过运动与分段质量检查 |
| `sync_calibration.fps` | 标定图像抽样 10～60 Hz，默认 30 Hz，且不得高于配置相机帧率；**不改变正常相机帧率** |
| `sync_calibration.max_mib` | 标定数据文件预算，64～65536 MiB，默认 2048 MiB；不包含原程序自己的录像文件 |
| `app.enable_timing_statistics` | 退出时在控制台打印流水线与相机耗时统计；场上配置默认 false，标定配置默认 true |

开始后：

1. 保持整机静止至少 **3 秒**，用于估计陀螺仪静态零偏。
2. 持续做平滑但变速的多轴往复转动，避免一直匀速或重复单一周期，棋盘格保持可见。
3. 保证光线足够，避免运动模糊，适当倾斜观察棋盘格，减少正对平面时的姿态歧义。
4. 等待定时结束。`Ctrl+C`／SIGTERM 会停止并清理线程，但本次标记为不完整，不能用于接受标定。

640×480、30 Hz、45 秒的未压缩灰度图约 415 MB，另有 CSV 与原程序输出。图像队列默认 8 帧、IMU 队列 4096 条；队列满不会等待磁盘，但会让该次标定无效。应改善记录速度或降低标定图像抽样率，而不是扩大到无限队列。

## 4. 准备现有内参

提供翻转后正立图像对应的内参和畸变 JSON，分辨率必须与正式 `prepare` 的输出一致：

```json
{
  "image_orientation": "runtime_prepared_180",
  "image_width": 640,
  "image_height": 480,
  "camera_matrix": [[500.0, 0.0, 315.0], [0.0, 500.0, 238.0], [0.0, 0.0, 1.0]],
  "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0]
}
```

**以上数值仅展示文件结构，不是本相机的标定结果，不能直接拿来标定时延。** 将矩阵和畸变替换为你的实测结果；不能因为配置文件只有矩阵，就假设镜头畸变为零。畸变即使确实为零，也须显式填写。支持 OpenCV 的 4、5、8、12、14 项畸变向量。

若 SDK 实际尺寸与正常处理尺寸不同，脚本会先复现正式程序的缩放，再翻转；不把原 SDK 尺寸误当作内参尺寸。

## 5. 离线分析

例如棋盘格为 9×6 个**内角点**，每个方格边长 30 mm：

```sh
python3 scripts/calibrate_camera_imu_sync.py \
  --data data/sync_capture_01 \
  --intrinsics /path/to/upright_intrinsics.json \
  --columns 9 --rows 6 --square-size-m 0.03
```

可选：`--max-delay-ms 100` 指 ±100 ms 搜索范围，允许 1～500 ms；`--stationary-seconds 2` 指用于估计零偏的起始静止段；`--max-segment-spread-ms 5` 指三个分段偏移的最大允许跨度。不要为得到“通过”结果任意放宽质量门。

脚本只接受 version=2 的完整采集，核对字节范围、时间顺序、补偿关系、计数和方向；不提供旧独立采集格式的兼容解析。

结果写入采集目录的 `calibration_result.json`。接受返回 0；无效数据／不可靠估计返回非零，不给出 `suggested_config`，但保留已获得的诊断。不会修改配置文件或原数据。

## 6. 解释结果

同一运动的原始相对偏移定义为：

```text
D_raw = camera_raw_timestamp - imu_raw_timestamp
D_runtime = D_raw + 当前 camera_timestamp_bias_ns + 当前 measurement_latency_ns
```

其中 `raw` 是已经统一到单调时钟、撤销固定补偿后的时间，不是 MCU 的真实采样时间，也不是 SDK 原始 timeval。原程序运行仍保留配置中的补偿，只有离线分析做逆补偿。

- `delay_ms`：D_raw；正数表示同一运动的相机时间戳比 IMU 时间戳晚。
- `runtime_delay_ms`：当前配置补偿后还剩多少相对偏移。
- `segment_delay_ms`／`segment_spread_ms`：三个时间分段的偏移及跨度。**跨度不是逐样本同步误差标准差，也不是置信区间。**
- `rmse_rad_s`／`correlation`：视觉与陀螺旋转幅度的匹配质量。
- `data_quality`：实际采样间隔、相机回调年龄、相机／IMU 数据可用延迟的经验分位数。这些不等于绝对采样时间误差；估计时间域下也可能出现负年龄。
- `recording`：抽样、接受／写入／丢弃计数、队列高水位、拷贝／灰度转换／写调用耗时及实际字节数。写调用时序是主机调用耗时，不是掉电可恢复的介质持久化承诺。
- `runtime_timing`：各原有阶段的实际执行次数和时序；`guidance.samples=0` 就不能声称测量了该分支的负载。

建议值保留现有 IMU latency 不变，仅调整相机偏置：

```text
camera_bias_new = -D_raw - 当前 measurement_latency_ns
```

只有数据和固定偏移模型均通过质量检查才会提供建议。需要人工审核后再决定是否修改生产配置。

## 7. 数据文件与失败排查

| 文件 | 内容 |
|---|---|
| `metadata.json` | 完成状态、结束原因、原补偿、原图／分析方向、实际 SDK 参数、接收／记录诊断和流水线统计 |
| `frames.gray8` | SDK 原方向、紧凑排列、未压缩的灰度像素；由 CSV 按 offset／length 索引 |
| `frames.csv` | 图像序号、运行／未补偿／回调／交付时间（整数 ns）、来源、字节范围与尺寸 |
| `imu.csv` | 全速原始解码加速度、角速度、舵机字段、运行／未补偿／交付时间 |
| `calibration_result.json` | 离线接受或拒绝报告 |

只有到时正常结束、记录文件关闭成功、诊断健康，才原子更新 `complete=true`。记录溢出、写盘失败、预算超限、异常时间戳、IMU 解码／接收丢失或中断均拒绝。保留不完整目录便于排查，但不能手工把 complete 改为 true。

IMU 接收统计以接收缓冲区关闭为边界；关闭后晚到的串口回调不纳入本次接收／丢失计数，避免正常停机被误判为采集丢失。关闭前的无效数据和真实队列溢出仍保留错误计数，并拒绝本次标定。

分段不稳定或最优值落搜索边界时，先检查运动激励、内参、棋盘格固定情况、接收线程实时优先级、采样间隔和丢失计数。若真实工作条件下偏移持续变化，应改进采样时间戳／接收链路，不能用一个固定补偿掩盖。
