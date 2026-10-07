# MOSAS Guidance 架构设计

> 状态：设计草案，等待逐阶段实现验收。本文是长期维护的架构契约，不代表源码已经按目标架构改造。
>
> 基线：`HEAD 93db8b4`。编写时工作树存在未提交修改；这些改动不作为本文行为基线，也不应被本设计覆盖或清理。

## 1. 目标与约束

### 1.1 目标

- 保留已验证制导行为，以渐进重构降低模块间耦合。
- 让视觉、姿态/起飞状态、滤波、制导律、传感器适配和输出模块可独立替换、单元测试。
- 将线程、队列、数据所有权、时钟和错误语义变成明确契约。
- 让日志、录制、无线显示等观测功能不能阻塞实时控制链。
- 预留硬件触发同步扩展点，但本阶段不改冻结的 IMU 串口协议。

### 1.2 固定系统约束

| 项目 | 约束 |
|---|---|
| 目标平台 | A133P，Ubuntu 22.04 |
| 主程序 | C++17 为主，协议/低层代码可用 C；工具脚本可用其他语言 |
| 相机 | Nori SDK，640×480，目标 120 FPS |
| IMU | 下位机串口协议冻结，固定 200 Hz，不含 MCU 采样时间或样本序号 |
| 控制 | 过载由上位机制导，下位机闭环驱动舵机 |
| 进程模型 | 单进程、多线程；模块在启动时组装，不做运行中热切换 |
| 坐标 | 镖体系过载；局部导航系原点为发射点，+X 沿初始发射方向的水平投影，+Y 向上，+Z 向右 |
| 图像时间 | 使用 SDK `Frame_Time`；控制链统一到 `CLOCK_MONOTONIC` |
| 日志时间 | 面向人的日志显示 `CLOCK_REALTIME`；内部耗时和排序仍用单调时钟 |

实时初始目标为每个有效相机帧产生一次制导更新，120 FPS 对应约 8.33 ms 周期。目标板需分别测量“采集到命令提交”和“采集到串口写完”的延迟；两者不可混为一个指标。8.33 ms 是首轮目标，不应在没有 A133P 实测前宣称为硬实时保证。

### 1.3 行为保持原则

第一阶段只调整模块边界、依赖方向和测试接缝，不改视觉判定、姿态估计、起飞检测、滤波、PNG 参数、坐标换算或串口字节协议。任何行为变化单独提出、单独测试和验收。

`HEAD 93db8b4` 是行为基线。当前工作树的未提交修改不纳入基线；尤其不得把其中对无效制导量的处理变化悄悄混入结构重构。下位机在没有新命令时保持旧指令还是进入其他状态，也必须先做协议/硬件确认，不能推断为安全。

## 2. 当前架构评估

项目已经有实用的模块基础，不需要推倒重写：

- CMake 已将 `serial_package`、`mosas_vision`、`mosas_guidance`、`mosas_runtime`、`mosas_output`、`mosas_app` 和可选 `mosas_wireless` 分成静态库。
- 已有 `ImuSource`、`CameraSource`、`CommandSink`、`FrameSink` 等边界；串口控制输出使用异步提交，日志也有异步队列。
- Nori SDK、串口协议解码、视觉识别、PNG、姿态估计、录像/CSV、RTSP 已有独立实现文件。

主要耦合点：

1. `GuidanceRuntime` 同时编排传感器读取、姿态/起飞估计、时间历史、视觉、LOS、滤波、PNG、命令提交、记录和生命周期。
2. runtime 数据类型和接口穿透 `cv::Mat`、`VisionResult` 等视觉实现类型；指导算法目标也直接依赖视觉库类型。
3. `AttitudeEstimator` 同时维护姿态和实际起飞状态；替换其中一个算法时，容易牵动另一个状态。
4. `FrameSink` / `OutputController` 汇集可视化、RTSP、视频、CSV 和 IMU 记录；单一 sink 的错误语义容易让可选输出影响运行时。
5. 当前 CMake 没有注册 CTest 测试目标。构建目录中的旧测试产物不作为可重复验收证据。
6. SDK 头文件将 `Frame_Time` 标为采集时间戳，但未声明其时钟域。当前配置中的时钟域、相机偏置和 IMU 延迟都要实机标定。

## 3. 目标架构

```text
                         ┌──────────────────────────┐
                         │ App / Composition Root   │
                         │ 配置校验、Factory、组装    │
                         └────────────┬─────────────┘
                                      │ 注入接口
       ┌──────────────────────────────┼──────────────────────────┐
       ▼                              ▼                          ▼
┌───────────────┐             ┌────────────────┐          ┌────────────────┐
│ Device Ports  │             │ Algorithm Ports│          │ Output Ports   │
│ Camera / IMU  │             │ Vision / State │          │ Command / Log  │
│ Clock / Sync  │             │ Filter / PNG   │          │ Video / Telemetry│
└───────┬───────┘             └───────┬────────┘          └───────┬────────┘
        │                             │                           │
        ▼                             ▼                           ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ Application Runtime                                                     │
│ RuntimeSupervisor + ImuPipeline + FramePipeline + OutputDispatcher       │
│ 有界队列、时间匹配、状态快照、故障监督；不依赖 Nori/FFmpeg/串口细节       │
└──────────────────────────────────────────────────────────────────────────┘
        ▲                             ▲                           ▲
        │                             │                           │
┌───────┴────────┐           ┌────────┴─────────┐        ┌────────┴─────────┐
│ Nori/Serial    │           │ Existing/Next    │        │ RTSP/Recorder/   │
│ Hardware Adapters│         │ Algorithm Impl.  │        │ Async Logger     │
└────────────────┘           └──────────────────┘        └──────────────────┘
```

依赖方向保持单向：

```text
Domain Types ← Ports ← Runtime/Application
       ↑          ↑             ↑
 Algorithms   Adapters      Composition Root
```

Domain/Ports 不依赖 Nori SDK、串口类、FFmpeg、OpenCV UI 类型或具体算法。实现初期可保留现有 CMake target；先建立源码和依赖边界，只有在测试/构建确实获益时再拆分 target。

## 4. 模块职责

| 模块 | 唯一职责 | 替换/测试边界 |
|---|---|---|
| `CameraSource` | 获取拥有生命周期的图像帧及 SDK 时间戳、接收时间 | Nori 实现 / Fake / 回放 |
| `ImuSource` | 接收原始字节、解码固定协议并产生带时间质量的 IMU 样本 | 串口实现 / Fake / 文件回放 |
| `TimeSync` | SDK 时钟域转单调时钟；IMU 接收时间回推；报告质量和不确定度 | 软件估时 / 将来的硬触发同步 |
| `VisionDetector` | 单帧目标识别，输出像素目标观测；不访问串口、日志或界面 | 当前识别器 / 替代识别器 |
| `FlightStateEstimator` | 每个 IMU 样本输出姿态快照与实际起飞状态 | 第一版适配现有组合式 `AttitudeEstimator`，避免重复消费 IMU |
| `AttitudeEstimator` / `LaunchDetector` | 可独立替换的姿态和起飞判据子模块 | 在行为回归通过后拆开组合；起飞判据先保持不变 |
| `LosEstimator` | 像素观测、相机内参和固定外参转换为镖体系 LOS | 纯逻辑单测 |
| `LosRateFilter` | 使用带时间戳 LOS 与同步角速度估计 LOS 角速度 | 当前 Kalman / 其他滤波器 |
| `GuidanceLaw` | 接收完整状态快照，计算镖体系过载与诊断量 | 当前 PNG / 替代制导律 |
| `CommandSink` | 非阻塞接收最新有效命令；串口写只在自己的发送线程执行 | 串口 / 日志 / Fake |
| `RecordingSink` | 录制视频、IMU 和逐帧制导遥测 | 本地文件 / Fake |
| `VisualizationSink` | RTSP 或其他可视化；不能反压控制链 | RTSP / Fake / 禁用 |
| `AsyncLogger` | 收集结构化日志并异步格式化/写出 | 控制台 / 文件 / Fake |
| `RuntimeSupervisor` | 启停、线程生命周期、故障汇总和 HealthSnapshot | Fake components 集成测试 |

所有实现由 App 在启动时通过 Factory/依赖注入选择。无需动态库插件，也不在飞行/控制运行中替换有状态算法。

## 5. 线程、队列与实时边界

```text
Nori callback
  └─ 立即记接收单调时间、保留 Frame_Time、取得/复制拥有型 buffer、入有界队列
       ↓
Camera capture/prepare worker
  └─ 解码和预处理；队列满时按“保留最新帧”丢旧帧并计数
       ↓
FramePipeline（单线程，维持跟踪器/滤波器调用顺序）
  ├─ 按采集时间查找 IMU/姿态快照
  ├─ Vision → LOS → LOS rate/filter → GuidanceLaw
  ├─ 非阻塞提交最新有效命令
  └─ 非阻塞提交不可变观测事件

UART receive thread
  └─ 只读取字节、记录首字节接收单调时间、放入 SPSC 原始块队列
       ↓
IMU decode/state worker（200 Hz 输入）
  ├─ 解码/CRC/延迟估计
  ├─ FlightStateEstimator（首版适配现有 AttitudeEstimator）
  └─ 更新固定容量时间历史

Command mailbox (capacity 1, latest wins)
  └─ Command worker → 编码现有冻结帧 → UART write

Observation queues (bounded, independent)
  ├─ Video/RTSP worker
  ├─ Local recording + CSV workers
  └─ Structured logging worker
```

线程契约：

- SDK 回调和实时处理路径不做磁盘/网络 I/O，不做同步日志写出。
- `process/update/try_submit/try_record` 必须有界且不可等待慢消费者；组件构造、配置、`start/stop` 只由 supervisor 调用，允许阻塞。
- 状态型视觉跟踪器、滤波器和制导实现由单一 FramePipeline 线程调用；IMU 姿态估计由唯一 IMU worker 调用。
- IMU 历史由 IMU worker 写、FramePipeline 读；使用已有单写多读快照/历史实现或明确的无锁/锁策略，不跨模块共享可变内部对象。
- 相机帧和处理结果用有界队列；控制命令用 latest-value mailbox，不排队发送过期控制量。
- 视频/无线/普通遥测是 best-effort 独立 sink；满队列时丢弃并累计诊断，不阻塞控制。录像失败是否影响制导的目标策略按可选观测输出处理，但第一阶段先通过回归测试保留 HEAD 行为，再单独切换故障策略。

## 6. 时间同步与硬触发扩展

控制计算的规范时间轴是 `CLOCK_MONOTONIC`。`CLOCK_REALTIME` 只用于日志显示/会话命名，日志事件同时保存单调时间以支持延迟计算和跨线程排序。

### 当前软件时间模式

- 相机：保存 SDK 原始 `Frame_Time`、SDK 域、主机回调接收单调时间。经显式映射后得到规范的单调采集时间；不允许静默用接收时间冒充采集时间。
- IMU：协议无设备时刻和样本序号，保存串口首字节接收单调时间，并减经实测的固定测量/传输偏置。该时间必须标为 `estimated`，并报告未知/已测的不确定度。
- IMU 200 Hz 的 5 ms 周期用于诊断和对积压作受限估计，不能恢复协议未提供的样本时刻或丢失序号。
- `Frame_Time` 时钟域、相机曝光参考偏置、IMU 测量到主机接收延迟均列为目标板校准项目。配置默认值不能被当作已校准值。

### 未来硬触发

Nori SDK 有外部硬件触发模式入口。保留 `ISyncProvider`/`ITriggerEventSource` 扩展点；当前只实现软件映射。未来要声称相机和 IMU 硬同步，触发事件/相位必须同时对下位机 IMU 时间模型可见。仅触发相机、下位机仍只发无序号串口帧，不足以证明 IMU 与曝光硬同步。物理连线和 MCU 对触发脉冲的观测方式留待硬触发阶段设计，不改变已冻结 UART 帧。

## 7. 坐标与物理量规范

- `DartFrame D`：`+X` 向前、`+Y` 向上、`+Z` 向右。
- `NavigationFrame N`：原点为发射点；`+X` 是初始发射架方向的水平投影（不将发射倾角写入导航系）；`+Y` 向上；`+Z` 向右。发射倾角体现为镖体系相对导航系的姿态。
- `BallisticFrame B`：`+X` 沿速度方向；小攻角模型下与镖体系仅相差绕纵轴的 roll。
- roll 正方向按绕 `+X` 的右手规则；坐标变换用带方向的 `R_target_from_source`，满足 `v_target = R_target_from_source * v_source`。
- `CameraFrame C` 到 `D` 保留静态旋转外参，默认单位矩阵；`D_from_C` 是唯一外参方向命名。坐标轴相同不等同于未经标定的内参正确。
- LOS 角/角速度统一用 rad、rad/s；加速度用 m/s²；过载用以 `g` 为单位的无量纲数值。结构字段名中携带单位和坐标系。
- 当前源代码的控制帧映射作为行为基线记录：roll 槽为 0、body-z 映射 yaw、body-y 映射 pitch，过载按 0.01 g 量化。重构中不得改字节/符号；目标板须与下位机协议再核对一次。

## 8. 故障与兼容边界

- 每条输入携带状态/有效性；状态至少能区分 `valid`、`stale`、`invalid`、`missing` 和 `source_fault`。
- 无有效观测不等同于合法零过载。接口分别表达 `invalid/no command` 与 `valid zero command`。
- 第一阶段逐字节/逐行为兼容 HEAD。任何无效输入下的清零、保持、停发或 fault 策略变化，要作为独立决策并和下位机看门狗/保持行为一起验收。
- 可选观测 sink 的故障有独立 health 状态；控制串口发送失败属于控制链故障，须留出清晰 fault code 与诊断快照。
- 队列丢弃、CRC 错误、时间回退、时间不确定度、帧龄、命令替代/过期、录制丢帧都必须可观测。

## 9. 渐进实施与验收路线


### 阶段 1 — 测试基础与领域契约

- 在 CMake/CTest 中建立不依赖 Nori arm64 `.so` 的 host 单测目标。
- 增加强坐标/单位、时间戳来源/质量、Observation/State/Command 的领域类型。
- 给坐标变换、相机时钟映射、IMU 延迟模型、协议解码写纯逻辑测试。
- 先适配旧结构，不同时重写算法。

**验收门**：x86_64 host 可构建核心逻辑和 Fake；坐标/单位/时钟用例全通过；目标机输出无变化。

### 阶段 2 — 算法接口注入

- 为 Vision、Attitude、LaunchDetector、LOS filter 和 GuidanceLaw 定义窄接口。
- 用现有实现作默认实现，通过 Composition Root 注入；禁止在算法里直接触碰硬件、日志或录像。
- 增加确定性 fake clock、fake IMU/camera 和各接口 contract tests。

**验收门**：同一输入 replay 的视觉结果、姿态/起飞状态、LOS、滤波结果和控制命令与阶段 0 基准一致。

### 阶段 3 — 拆分 Runtime 编排

- `GuidanceRuntime` 逐步拆成 `RuntimeSupervisor`、`ImuPipeline`、`FramePipeline`、`OutputDispatcher`。
- 保留单进程和线程职责，不并行调用有状态算法。
- 数据流改成不可变快照；配置验证和组件 Factory 由 App composition root 负责。

**验收门**：Fake 源完成完整流水线测试；start/stop/cancel/fault/restart 无死锁、越界或跨代命令。

### 阶段 4 — 观测输出隔离

- 将当前综合 FrameSink 拆为互相独立的录像、遥测、无线显示、IMU 记录和日志 sink。
- 每个 sink 单独有界队列、丢弃计数、错误状态；慢 sink 不反压控制。
- 先保持 HEAD 错误行为，之后再单独启用“可选输出失败不停止制导”的策略并验收。

**验收门**：禁用/阻塞/故障任意一个可选 sink，不增加控制处理延迟，不污染其他 sink；丢弃和错误可观测。

### 阶段 5 — A133P 时序与同步验收

- 确认 SDK `Frame_Time` 时钟域和曝光参考点；测出时钟映射漂移/偏置。
- 用逻辑分析仪或同步 GPIO 测量 IMU 测量时刻到 UART 首字节/主机接收的延迟和抖动，填写补偿值。
- 在 640×480、120 FPS、IMU 200 Hz、日志/录像/无线满负载下记录处理阶段 P50/P95/P99/max、队列峰值/丢弃、CPU/内存。
- 检验 capture→command-submit 首轮目标约 8.33 ms；同时报告 UART write completion 延迟，不把提交时间当作物理到达时间。
- 硬触发实现另开阶段；先确认相机脉冲和 MCU/IMU 对共同触发的观测线路，再开发 `ISyncProvider` 实现。

**验收门**：时间质量、对齐误差和延迟指标有实测报告；触发失败、时钟异常、串口积压、丢帧恢复和输出满载均按状态契约处理。

## 10. 本设计中的待实机确认项

1. `Frame_Time` 实际时钟域及在 A133P 重启/NTP 校时后的映射行为。
2. 相机采集时间相对曝光参考点的偏置。
3. IMU 测量时刻到串口首字节和主机接收时间的固定延迟/抖动；协议没有 MCU 时间戳或样本序号。
4. IMU/相机同步允许误差门限；首轮先采集数据，再由实测确定门限。
5. 下位机在无新过载命令、主机退出或串口断线时的实际行为。
6. 未来硬触发脉冲如何同时进入相机及 MCU/IMU 时间模型；UART 协议维持冻结。
