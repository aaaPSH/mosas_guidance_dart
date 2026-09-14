# MOSAS Guidance 架构审查报告

> 审查日期：2026-09-11  
> 审查范围：项目目录、CMake 构建文件、主入口、核心模块、接口定义、配置文件、线程与通信代码、测试代码及辅助脚本。  
> 审查性质：静态架构审查，不等同于实机可靠性认证。

## 结论摘要

当前项目是一个单进程、多线程、流水线式的视觉制导系统。总体架构方向合理，已经具备硬件接口抽象、有界队列、状态码、异常捕获和 Fake/Mock 测试基础。

但当前架构还不能认为是低风险、易扩展的生产级架构。主要风险集中在：

1. GuidanceRuntime 同时承担线程调度、状态估计、视觉识别、制导计算、控制输出、异常处理、日志和统计。
2. 控制指令发送和同步日志进入实时处理路径，可能造成不可预测阻塞。
3. 时间戳、数据年龄、状态所有权和安全降级策略不够统一。
4. RtspFrameSink 同时负责推流、录像、CSV、可视化和调试数据。
5. 测试目录、CMake 和源码接口已经出现漂移。

建议保留单进程、多线程和 bounded queue 的总体形态，不建议引入微服务、事件总线或大型依赖注入框架。后续应采用小步重构，优先处理实时性、安全降级、时间体系和测试闭环。

## 审查证据和验证结果

- 当前主工程的模块和调用关系已通过源码、头文件、配置及脚本检查。
- tests/ 和 docs/ 物理存在，但被 .gitignore 忽略；其中部分测试与当前接口不一致。
- 当前主 CMakeLists.txt 没有注册 enable_testing() 或 add_test()。
- 当前主机为 x86_64，CMake 配置在 CMakeLists.txt:15 因 Nori SDK 仅支持 ARM64 而失败。
- python3 scripts/vision_replay_test.py 的 7 个测试通过。
- 本报告不把旧 build/ 目录中的缓存测试目标当作当前源码的测试结果。

---

# 第一部分：当前架构概览

## 1. 主要模块

| 模块 | 主要职责 | 评价 |
|---|---|---|
| serial_package | 串口读写、控制帧和 IMU 帧协议、CRC | 边界相对清晰 |
| mosas_dart | 姿态和速度积分、Dart 内部状态 | 有独立状态机 |
| mosas_vision | OpenCV 图像处理、ROI、目标检测、调试数据 | 算法和调试数据耦合 |
| mosas_guidance | LOS、LOS 角速度和 PNG 制导 | 算法接口相对清晰 |
| mosas_runtime | 多线程调度、IMU 状态更新、图像处理、故障处理 | 职责过重 |
| mosas_app | 配置、Nori 相机适配、串口 IMU、控制适配 | 平台适配较集中 |
| mosas_wireless | RTSP、录像、CSV、可视化 | 输出职责过多 |
| mosas_guidance_app | 配置加载、组件组装、启动和停止 | Composition Root 基本合理 |

## 2. 物理依赖关系

~~~text
mosas_guidance_app
        ↓
    mosas_app
     ↓   ↓   ↓
 runtime serial  Nori SDK
    ↓
 guidance
    ↓
 dart + vision
~~~

无线输出路径：

~~~text
mosas_app
    ↓
mosas_wireless
    ↓
FFmpeg / OpenCV
~~~

当前没有发现明确的：

~~~text
A → B → C → A
~~~

形式的 CMake 目标循环依赖。

但存在层级泄漏：FrameSink 接口位于 runtime，wireless 模块实现该接口，因此 mosas_wireless 在概念上依赖运行时层，而不是独立的 output port 层。

## 3. 主要数据流

### IMU 数据流

~~~text
串口
  ↓
SerialPort
  ↓
SerialImuSource
  ↓
串口帧解析 / CRC 校验
  ↓
主机时间戳或积压时间重建
  ↓
FlightPhaseDetector
  ↓
DartCondition
  ↓
ImuStateSnapshot
  ↓
ImuStateHistory + latest_state_
~~~

### 相机、视觉和制导数据流

~~~text
Nori SDK callback
        ↓
原始帧复制到 pending queue
        ↓
NoriSdkCameraSource::capture()
        ↓
MJPG/YUYV 解码
        ↓
resize / undistort
        ↓
VisionRecognizer
        ↓
LOS 计算
        ↓
LOS 角速度滤波
        ↓
PNG 制导
        ↓
CommandSink
        ↓
串口或日志输出
~~~

processing worker 根据相机时间戳查找不晚于该时间的 IMU 状态，并检查 max_imu_age 和 FlightPhase。

### 输出数据流

~~~text
ProcessedFrame
     ↓
output_queue
     ↓
RtspFrameSink
     ├── RTSP 推流
     ├── 本地录像
     ├── CSV 飞行数据
     ├── 可视化叠加
     └── 视觉调试图像
~~~

## 4. 数据所有权

当前主要所有权关系为：

~~~text
main
 └── GuidanceRuntime
      ├── ImuSource
      ├── CameraSource
      ├── CommandSink
      ├── FrameSink
      ├── FlightPhaseDetector
      ├── DartCondition
      ├── ImuStateHistory
      └── bounded queues
~~~

unique_ptr 使主要组件的生命周期较清晰，这是优点。

但是系统没有唯一的“飞行状态所有者”：

- FlightPhaseDetector 保存飞行阶段。
- DartCondition 保存姿态、速度和自身状态。
- GuidanceRuntime 保存 initialized_、launched_、latest_state_。
- ImuStateHistory 保存历史状态。

当前这些对象主要由 IMU worker 修改，因此暂时没有确认的多线程同时写入同一状态问题；但语义上存在多个状态源，未来增加恢复或重连逻辑时容易出现不一致。

SerialPort 通过 shared_ptr 同时被 IMU 输入和控制输出使用。当前单读线程、单写线程可以工作，但自动重连、多控制输出或多 runtime 场景下风险会增加。

## 5. 控制流

~~~text
main
  ↓
GuidanceRuntime::start()
  ├── IMU worker
  ├── camera capture worker
  ├── processing worker
  └── output worker

IMU worker
  └── 更新阶段、姿态、速度和历史状态

capture worker
  └── 获取相机帧并写入 capture_queue

processing worker
  ├── 读取相机帧
  ├── 查找 IMU 状态
  ├── 视觉识别
  ├── 制导计算
  ├── 发送控制指令
  └── 写入 output_queue

output worker
  └── RTSP / 录像 / CSV / 可视化

任意 worker 异常
  ↓
set_fault()
  ↓
cancel source、关闭 queue、join 线程
~~~

正常调用关系中没有发现 A 控制 B、B 再反向控制 A 的直接循环。Nori callback 也基本只负责复制和入队，没有把复杂视觉处理放进 SDK callback。

## 6. 线程模型

当前线程包括：

1. 主线程：配置、启动、等待、停止和状态打印。
2. Nori SDK callback 线程：接收原始图像并入队。
3. IMU worker：串口读取、协议解析、阶段判断和状态估计。
4. Camera capture worker：相机采集和解码。
5. Processing worker：视觉、LOS、PNG 和控制指令。
6. Output worker：网络、磁盘、录像、CSV 和可视化。

相机名义频率为 120 Hz，IMU 名义频率为 500 Hz，RTSP/录像名义频率为 30 Hz。控制线程没有独立周期配置，实际控制频率由有效相机帧、IMU 匹配、视觉识别和制导输出共同决定。

---

# 第二部分：架构优点

## 1. Composition Root 清晰

main 负责加载配置、创建硬件适配器、组装 GuidanceRuntime 并管理生命周期，没有引入大型容器或复杂事件总线。

## 2. 硬件接口具备替换基础

ImuSource、CameraSource、CommandSink 和 FrameSink 使硬件实现与 runtime 之间存在基本隔离。Fake source 和 test sink 可以用于离线测试。

## 3. Nori SDK 有独立适配层

NoriSdkApi 支持生产实现和 Fake 实现分离，视觉算法不直接调用厂商 SDK。

## 4. 队列有界并采用丢旧策略

LatestFrameQueue 满时丢弃旧帧，避免输入积压无限增长。对于相机图像，这通常比处理几十毫秒以前的图像更合理。

## 5. 停止顺序经过设计

运行时会先取消 source，再等待采集线程退出，之后停止相机、关闭队列并等待其它线程，避免在 SDK 图像转换尚未完成时释放资源。

## 6. 错误状态和异常传播基本完整

SourceStatus 区分 ok、timeout、fatal 和 cancelled。worker 捕获异常并进入 set_fault()，串口协议和 CRC 也独立实现。

## 7. 配置管理基础较好

配置集中，支持默认值、重复 key 检测、基础类型解析、范围检查和相对路径处理。

## 8. 没有明显过度设计

当前没有不必要的微服务、事件总线、多层 Manager 或过深继承层级，适合继续采用小步重构。

---

# 第三部分：架构问题

## P0：当前没有足够证据确认 P0

当前没有证据确认已经发生死锁、已确认的数据竞争或控制指令失控。

但是，如果下位机在没有新指令时会持续保持上一条控制指令，则以下 P1 风险可能升级为 P0：

- 控制指令发送阻塞实时链路；
- 没有统一的过期指令策略；
- 传感器断开后没有明确 safe command；
- 输出失败可能导致状态处理不一致。

下位机控制帧超时和安全行为需要结合硬件协议进一步验证。

## P1-1：控制输出位于实时关键路径

- **问题：** processing worker 直接调用 command_sink_->send()。
- **具体代码位置：** src/runtime/guidance_runtime.cpp 的 processing worker 和 command_sink_->send()；src/app/host_adapters.cpp 的 SerialCommandSink::send()；src/serial_package/serial_port.cpp 的 write_all()。
- **产生原因：** CommandSink 是同步接口，没有独立控制队列和控制输出线程。
- **可能后果：** 串口写入延迟会暂停视觉处理，导致相机帧丢弃、控制周期变长和输出滞后。
- **触发条件：** 串口缓冲区满、USB/串口异常、设备响应慢，或未来将控制输出改为网络。
- **建议修改方式：** 增加 bounded control mailbox 或 SPSC queue，实时线程只提交固定大小命令；控制线程负责串口写入、sequence 检查、过期检查和 watchdog。
- **是否需要立即修改：** 需要，在实机控制和实时性验收前处理。

## P1-2：同步日志可能阻塞实时线程

- **问题：** GuidanceRuntime::log() 和 SerialImuSource::log() 使用同步 ostream 输出，并持有日志 mutex。
- **具体代码位置：** src/runtime/guidance_runtime.cpp 的 log()、set_error()；src/app/serial_imu_source.cpp 的 log()。
- **产生原因：** 实时 worker 直接格式化字符串并写入 console 或文件流。
- **可能后果：** IMU 或 processing worker 受到系统调用、内部锁、缓冲刷新和慢终端影响。
- **触发条件：** IMU timeout、串口积压、相机异常、连续目标丢失或输出重定向到慢文件系统。
- **建议修改方式：** 实时线程只提交固定大小 LogEvent，后台日志线程负责格式化、console、文件和 flush。
- **是否需要立即修改：** 需要。

## P1-3：GuidanceRuntime 职责过重

- **问题：** GuidanceRuntime 同时负责线程调度、状态估计、飞行阶段、视觉识别、LOS、PNG、控制输出、故障处理、日志和统计。
- **具体代码位置：** include/mosas/runtime/guidance_runtime.hpp 的配置和成员；src/runtime/guidance_runtime.cpp 的 processing worker。
- **产生原因：** 为快速实现完整流水线，将算法和调度直接写入 runtime。
- **可能后果：** 更换算法、增加算法或增加传感器时需要修改大型 runtime；测试边界变模糊。
- **触发条件：** 增加第二种算法、完整离线回放、新状态或新传感器。
- **建议修改方式：** 逐步抽取 StateEstimator、FlightStateMachine、FrameProcessor、CommandDispatcher 和 HealthMonitor。
- **是否需要立即修改：** 不需要立即大规模重写，但应列入第二阶段。

## P1-4：多个相关状态机没有统一所有者

- **问题：** FlightPhaseDetector、DartCondition 和 GuidanceRuntime 分别保存相关状态。
- **具体代码位置：** include/mosas/runtime/flight_phase_detector.hpp、include/mosas/dart/dart_condition.hpp、src/runtime/guidance_runtime.cpp 的 IMU worker。
- **产生原因：** 每个模块独立维护自己的初始化、起飞和飞行状态。
- **可能后果：** FlightPhase、DartCondition::State、initialized_ 和 launched_ 可能出现语义不一致。
- **触发条件：** 增加重连、异常恢复、重复启动或状态回退。
- **建议修改方式：** 由一个 FlightStateMachine 统一维护 Uninitialized、Ready、Ejection、FreeFlight、Safe 和 Stopped。
- **是否需要立即修改：** 增加恢复逻辑前需要处理。

## P1-5：传感器时间语义不足以证明时间对齐正确

- **问题：** 相机使用 callback 到达时间，串口 IMU 使用主机接收时间并根据积压量重建时间。
- **具体代码位置：** src/app/nori_camera_source.cpp 的时间戳生成；src/app/serial_imu_source.cpp 的 timestamp_for_frame()。
- **产生原因：** ImuSample 和 CameraFrame 主要只有 timestamp_ns，没有时间域、sequence、source 或时间质量。
- **可能后果：** 高负载和设备延迟下可能发生 IMU/相机错配；离线回放无法复现真实延迟。
- **触发条件：** 串口积压、SDK callback 延迟、队列堆积和传感器频率提高。
- **建议修改方式：** 增加 timestamp、sequence、clock_domain、source、valid 和数据年龄；增加最大相机帧年龄和最大处理延迟检查。
- **是否需要立即修改：** 在确认实机时间精度前必须进一步验证，建议列为第一阶段风险。

## P1-6：Nori 相机停止和 callback 生命周期不完全可控

- **问题：** callback context 被静态保存；部分 SDK 停止失败路径可能提前返回；等待 callback 没有明确超时；callback 中复制和分配大块图像。
- **具体代码位置：** src/app/nori_camera_source.cpp 的 callback context、stop() 和 enqueue_frame()。
- **产生原因：** SDK 没有可靠的 callback unregister 接口，资源清理和 SDK 生命周期被实现细节驱动。
- **可能后果：** 多次启动停止后内存增长、SDK 资源泄漏、停止永久等待、callback 延迟抖动。
- **触发条件：** 相机异常、SDK 停止失败、反复重启和高分辨率图像。
- **建议修改方式：** 使用完整资源清理 guard；停止失败时继续释放其它资源；增加等待超时；减少 callback 内分配和复制。
- **是否需要立即修改：** 长时间运行和反复重启前需要处理。

## P1-7：测试、CMake 和源码已经漂移

- **问题：** 当前主 CMake 不注册 C++ 测试，物理存在的测试部分使用已不存在的接口或头文件。
- **具体代码位置：** CMakeLists.txt、tests/serial_host_adapters_test.cpp、tests/nori_camera_diagnostic_test.cpp、.gitignore。
- **产生原因：** 测试目录和文档目录被忽略，测试没有随着接口变更同步维护。
- **可能后果：** 可能出现没有测试所以通过的假象，架构重构缺少回归保护。
- **触发条件：** 换机器、重新配置 CMake、修改接口或准备发布。
- **建议修改方式：** 将有效测试纳入 CTest；修复或删除过期测试；提供不依赖 ARM64 Nori SDK 的 host fake 构建；增加 CI 和 ThreadSanitizer。
- **是否需要立即修改：** 正式发布前需要处理。

## P1-8：可选无线输出失败可能使整个 runtime 故障

- **问题：** RTSP 失败且未启用录像时，RtspFrameSink 返回失败，output worker 可能触发 runtime fault。
- **具体代码位置：** src/wireless/rtsp_frame_sink.cpp 的无线失败处理；src/runtime/guidance_runtime.cpp 的 output worker。
- **产生原因：** FrameSink 只有单一 bool 结果，未区分关键输出和可选观测输出。
- **可能后果：** 网络断开可能停止制导，即使无线只是遥测用途。
- **触发条件：** RTSP 服务端断开、网络中断或编码器初始化失败。
- **建议修改方式：** 配置中区分 required 和 optional 输出；可选输出失败时保持控制链路运行并只更新 health 状态。
- **是否需要立即修改：** 如果无线不是控制必需项，则需要立即修正。

## P2-1：runtime 接口泄漏 OpenCV 和具体算法类型

- **问题：** runtime 数据类型直接包含 cv::Mat、视觉结果和制导结构。
- **具体代码位置：** include/mosas/runtime/runtime_types.hpp、include/mosas/runtime/runtime_interfaces.hpp。
- **产生原因：** 为减少转换直接复用视觉层对象。
- **可能后果：** 核心算法难以脱离 OpenCV 测试，替换图像库和输出后端困难。
- **触发条件：** 替换图像库、引入硬件加速或增加不同图像格式。
- **建议修改方式：** 将核心数据拆成 domain 类型，OpenCV 转换只放在视觉适配层。
- **是否需要立即修改：** 否，可在降低耦合阶段处理。

## P2-2：RtspFrameSink 职责过多

- **问题：** 一个类同时负责 RTSP、录像、CSV、resize、可视化和调试图像。
- **具体代码位置：** include/mosas/wireless/rtsp_frame_sink.hpp、src/wireless/rtsp_frame_sink.cpp。
- **产生原因：** 将所有输出能力合并进一个 sink。
- **可能后果：** 增加新输出时修改大类；录像、推流和可视化故障互相影响。
- **触发条件：** 增加网络遥测、抓帧、新录像格式或独立数据记录。
- **建议修改方式：** 拆分 RtspPublisher、VideoRecorder、TelemetryRecorder 和 FrameVisualizer，由 output worker 直接组合调用。
- **是否需要立即修改：** 中期处理。

## P2-3：串口生命周期和多 writer 契约较弱

- **问题：** fd_ 没有统一生命周期锁，write_mutex_ 只保护单次 write()，不能保证整个 write_all() 原子。
- **具体代码位置：** include/serial_package/serial_port.hpp、src/serial_package/serial_port.cpp。
- **产生原因：** 接口要求调用方保证不并发 close/read/write。
- **可能后果：** 自动重连、多输出线程或并发 close 时行为不可控。
- **触发条件：** 增加重连机制、第二个控制输出或多 runtime。
- **建议修改方式：** 明确单线程拥有串口，或在 SerialPort 内统一保护生命周期；write_all() 整体持有写锁。
- **是否需要立即修改：** 当前单 writer 场景不立即处理，但扩展前必须处理。

## P2-4：Nori SDK 使用进程级全局状态

- **问题：** SDK session 和生产实现使用全局状态及静态对象。
- **具体代码位置：** src/app/nori_camera_sdk.cpp 的 g_session_mutex、g_active_sdk、session reference 和静态 SDK 实例。
- **产生原因：** 厂商 SDK 需要进程级 init/uninit。
- **可能后果：** 多相机、多 runtime、不同 Fake SDK 并行测试受到限制。
- **触发条件：** 增加第二个相机、同进程多个 pipeline 或并行测试。
- **建议修改方式：** 保留进程级 SDK runtime，但由 app 层显式拥有，减少隐藏全局状态。
- **是否需要立即修改：** 否。

## P2-5：配置存在重复表示和分散校验

- **问题：** 相机内参同时存在于 camera_matrix 和 guidance intrinsics；相机配置同时存在 device 和 device_id。
- **具体代码位置：** include/mosas/app/app_config.hpp、src/app/app_config.cpp。
- **产生原因：** SDK 配置、算法配置和平台配置合并在一个 AppConfig 中。
- **可能后果：** 两套值不一致，配置字段可能被接受但实际不参与设备选择。
- **触发条件：** 更换相机、增加第二种相机或修改内参来源。
- **建议修改方式：** 解析原始配置后转换为唯一 normalized runtime config，由统一 validator 完成最终检查。
- **是否需要立即修改：** 否，但不应继续增加重复字段。

## P2-6：队列、延迟和丢帧状态没有完整暴露

- **问题：** 队列会丢旧帧，但公共统计没有完整暴露队列长度、丢帧数、数据年龄和处理延迟。
- **具体代码位置：** include/mosas/runtime/latest_frame_queue.hpp、GuidanceRuntime::statistics()。
- **产生原因：** 当前统计主要集中在 FPS 和平均/最大处理耗时。
- **可能后果：** 系统看似运行正常，但实际处理的是高延迟或低质量数据。
- **触发条件：** 输入频率提高、算法变慢、网络或磁盘输出变慢。
- **建议修改方式：** 增加 queue_size、dropped_frames、data_age、processing_latency 和 output_latency 指标。
- **是否需要立即修改：** 建议第一阶段加入。

## P2-7：重启后统计状态可能混入历史运行

- **问题：** runtime 重置了部分状态，但 FrameRateMeter 和 timing accumulator 没有统一 reset 语义。
- **具体代码位置：** src/runtime/guidance_runtime.cpp 的 start()；include/mosas/runtime/frame_rate_meter.hpp。
- **产生原因：** 队列和业务状态被重置，而统计对象跟随 runtime 生命周期。
- **可能后果：** 第二次运行的 FPS、耗时和最大值混入第一次运行数据。
- **触发条件：** 故障重启、测试中重复 start/stop、设备重新连接。
- **建议修改方式：** 增加统一的 RuntimeMetrics::reset()，每次启动前清除运行期统计。
- **是否需要立即修改：** 重启场景正式使用前建议处理。

## P2-8：视觉结果和调试数据耦合

- **问题：** VisionResult 同时保存识别结果、调试阶段和调试 mask；识别过程会复制调试图像。
- **具体代码位置：** include/mosas/vision/vision_types.hpp、src/vision/vision_recognizer.cpp。
- **产生原因：** 可视化直接复用了算法结果结构。
- **可能后果：** 高帧率下产生额外分配和复制，核心算法接口被 UI 需求污染。
- **触发条件：** 高分辨率、高帧率或关闭调试但仍执行同一路径。
- **建议修改方式：** 分离核心 VisionObservation 和可选 VisionDebugData，只在启用调试时生成 mask。
- **是否需要立即修改：** 性能压力测试前建议验证。

## P3-1：FPS 统计初始阶段语义不稳定

- **问题：** 样本较少时使用短窗口计算 FPS，启动阶段数值可能不稳定。
- **具体代码位置：** include/mosas/runtime/frame_rate_meter.hpp。
- **产生原因：** 没有定义最小有效样本数或 valid 状态。
- **可能后果：** 监控信息在启动阶段误导排障。
- **触发条件：** 刚启动或刚 reset。
- **建议修改方式：** 增加最小样本数和 valid 标志。
- **是否需要立即修改：** 否。

## P3-2：无效阶段检测配置可能静默回退默认值

- **问题：** FlightPhaseDetector 独立构造时可能把无效配置回退为默认配置。
- **具体代码位置：** src/runtime/flight_phase_detector.cpp。
- **产生原因：** runtime 外层已经有配置验证，内部实现选择容错。
- **可能后果：** 单独使用模块时配置错误不容易发现。
- **建议修改方式：** 统一使用显式错误返回或 factory 创建接口。
- **是否需要立即修改：** 否。

## P3-3：Python replay 与 C++ 视觉逻辑重复

- **问题：** scripts/vision_replay.py 有独立视觉处理实现。
- **可能后果：** 长期可能发生参数和行为漂移，离线结果与实机不一致。
- **建议修改方式：** 共享核心算法，或明确 replay 只是近似调参工具。
- **是否需要立即修改：** 否。

## P3-4：部分测试访问 private 成员

- **问题：** nori_camera_source_test.cpp 使用 #define private public 验证内部状态。
- **可能后果：** 测试绑定实现细节，后续重构时容易大量失效。
- **建议修改方式：** 增加必要的诊断接口，优先通过行为和 Fake SDK 验证。
- **是否需要立即修改：** 否。

---

# 第四部分：推荐架构

## 1. 总体原则

- 保留单进程和现有多线程模型。
- 保留 bounded queue。
- 不引入事件总线或微服务。
- runtime 只负责调度和监督，不直接实现所有算法。
- 控制输出和日志输出必须通过非阻塞提交与后台 I/O 解耦。

## 2. 推荐模块

~~~text
app
 ├── ConfigLoader
 ├── ConfigNormalizer
 └── CompositionRoot

ports
 ├── ImuSource
 ├── CameraSource
 ├── ControlOutput
 ├── FramePublisher
 ├── Clock
 ├── Logger
 └── Metrics

input
 ├── SerialImuSource
 ├── NoriCameraSource
 └── ReplaySource

state
 ├── StateEstimator
 ├── FlightStateMachine
 ├── StateSnapshot
 └── ImuStateHistory

vision
 ├── VisionDetector
 ├── VisionDebugData
 └── VisionVisualizer

guidance
 ├── LineOfSightEstimator
 ├── LineOfSightRateEstimator
 └── GuidanceAlgorithm

runtime
 ├── Input workers
 ├── FrameProcessor
 ├── CommandDispatcher
 ├── Output workers
 └── Supervisor

output
 ├── SerialCommandOutput
 ├── LoggingCommandOutput
 ├── RtspPublisher
 ├── VideoRecorder
 └── TelemetryRecorder
~~~

## 3. 推荐数据流

~~~text
IMU Source
   ↓
ImuSample(timestamp, sequence, source)
   ↓
StateEstimator
   ↓
FlightStateMachine
   ↓
StateStore / StateHistory
~~~

~~~text
Camera Source
   ↓
CameraFrame(timestamp, sequence, source)
   ↓
bounded latest-frame queue
   ↓
FramePreprocessor
   ↓
VisionDetector
   ↓
VisionObservation
   ↓
LOS / Rate Estimator
   ↓
GuidanceAlgorithm
   ↓
CommandDispatcher
   ↓
CommandOutput
~~~

## 4. 推荐线程模型

~~~text
SDK callback thread
    └── 只复制、打时间戳、入队

IMU thread
    └── 读取、解析、状态估计

Camera thread
    └── 读取、解码、预处理

Processing thread
    └── 视觉、制导、生成控制命令
        不执行串口、网络、磁盘和同步日志 IO

Command output thread
    └── sequence、stale、watchdog、串口/Socket I/O

Frame output thread
    └── RTSP、录像、CSV、可视化

Logging thread
    └── 格式化、console、file、flush
~~~

## 5. 数据所有权

~~~text
GuidanceRuntime
 ├── 拥有 worker、queue 和生命周期
FlightStateMachine
 └── 唯一拥有飞行阶段和安全状态
StateEstimator
 └── 唯一拥有姿态和速度内部状态
StateHistory
 └── 拥有历史 snapshot
CommandDispatcher
 └── 拥有最新控制命令和安全命令
各 Source
 └── 只拥有自己的设备缓冲区
~~~

## 6. 时间体系

关键数据应包含：

~~~text
timestamp
sequence
clock_domain
source
valid
age
~~~

所有关键路径都应检查：

- 是否乱序；
- 是否来自未来；
- 是否超过最大年龄；
- 是否超过最大处理延迟；
- 是否来自正确的数据源。

## 7. 状态机

推荐集中管理为：

~~~text
Uninitialized
    ↓
Ready
    ↓
Ejection
    ↓
FreeFlight
    ↓
Safe / Stopped

任意运行状态 ──故障──> Safe
~~~

状态切换需要具备：

- 明确的唯一写入者；
- 阶段确认时间；
- 超时和恢复策略；
- 目标丢失策略；
- 传感器掉线策略；
- 安全退出路径。

## 8. 异常传播

~~~text
Camera timeout
    ↓
Camera health degraded
    ↓
丢弃当前帧
    ↓
超过阈值
    ↓
Guidance disabled
    ↓
Safe command
~~~

~~~text
IMU disconnected
    ↓
StateEstimator invalid
    ↓
禁止继续使用旧状态
    ↓
CommandDispatcher safe state
    ↓
Supervisor 决定重连或停止
~~~

---

# 第五部分：重构优先级

## 第一阶段：必须立即修复

1. 移除控制输出在 processing worker 中的同步调用。
2. 增加 bounded command queue 或 latest-value mailbox。
3. 增加控制命令时间戳、sequence、过期判断和 safe command。
4. 移除实时线程同步日志和高频字符串格式化。
5. 增加队列长度、丢帧数、数据年龄和处理延迟统计。
6. 修复 Nori 停止失败路径的资源清理。
7. 明确可选无线输出失败是否允许影响制导。
8. 在真实硬件上验证 IMU 与相机时间延迟。

## 第二阶段：降低耦合

1. 从 GuidanceRuntime 提取 FrameProcessor。
2. 从 runtime 提取 StateEstimator。
3. 合并多个状态机，建立唯一状态所有者。
4. 拆分 RtspFrameSink。
5. 将配置解析结果转换为唯一 normalized runtime config。
6. 逐步减少 runtime 接口对 OpenCV 和具体算法类型的依赖。

## 第三阶段：提高可测试性

1. 将有效 C++ 测试加入 CMake/CTest。
2. 修复或删除过期测试。
3. 增加不依赖 ARM64 Nori SDK 的 host fake 构建。
4. 增加 FakeClock 和完整离线回放。
5. 增加串口、网络、磁盘和算法故障注入测试。
6. 增加 ThreadSanitizer 和长时间 soak test。

## 第四阶段：长期维护

1. 统一结构化日志和 HealthSnapshot。
2. 增加启动自检和退出故障摘要。
3. 优化 Nori callback 内存分配。
4. 显式管理 Nori SDK 进程级生命周期。
5. 统一 Python replay 与 C++ 核心算法，或明确两者差异。
6. 进行数小时到数天的持续运行测试。

---

# 第六部分：架构评分

评分标准：越高越好；“模块耦合程度”按“低耦合程度”评分。

| 指标 | 评分 |
|---|---:|
| 模块职责清晰度 | 6/10 |
| 模块耦合程度 | 5/10 |
| 数据流清晰度 | 7/10 |
| 数据所有权 | 6/10 |
| 接口设计 | 6/10 |
| 可测试性 | 6/10 |
| 可扩展性 | 5/10 |
| 实时性设计 | 5/10 |
| 线程安全 | 6/10 |
| 异常处理 | 6/10 |
| 日志与可观测性 | 5/10 |
| 配置管理 | 7/10 |
| 长期可维护性 | 5/10 |

总体评分：**5.8/10**。

最值得优先解决的三个问题：

1. 控制输出和同步日志进入实时关键路径。
2. GuidanceRuntime 职责过重，且状态机没有统一所有者。
3. 时间、数据年龄和健康状态模型不足，同时测试和 CMake 存在漂移。

## 最终判断

当前项目不是需要推倒重来的架构。它已经具备继续演进的基础，尤其是硬件接口、bounded queue、Fake source 和异常状态设计。

建议围绕以下目标做渐进式改造：

~~~text
低耦合
  ↓
单一状态所有者
  ↓
统一时间体系
  ↓
非阻塞控制提交
  ↓
异步日志和可观测性
  ↓
可重复的离线和实机测试
~~~

