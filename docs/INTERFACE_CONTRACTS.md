# MOSAS Guidance 接口契约

> 对应架构：[`ARCHITECTURE_DESIGN.md`](ARCHITECTURE_DESIGN.md)。
>
> 本文定义目标接口形状和语义，不是可直接编译的头文件。所有接口默认由 App 在启动时创建并注入；不引入运行期热插拔。第一阶段必须以 `HEAD 93db8b4` 的行为为回归基准。

## 1. 公共约定

### 1.1 实时调用规则

| 调用类型 | 是否可阻塞 | 说明 |
|---|---:|---|
| SDK callback、`try_pop`、算法 `process/update`、`try_submit`、`try_record` | 否 | 有界工作量，不做串口/磁盘/网络 I/O，不格式化文本日志 |
| `start/configure/stop/join` | 可以 | 仅由 `RuntimeSupervisor` 在非制导线程调用 |
| 诊断格式化、Wall-clock 字符串、CSV/视频编码、RTSP | 可以 | 专属后台 worker 执行 |

高频接口返回 enum/status 和固定大小诊断值；错误字符串、异常上下文由非实时监督层生成。状态型算法有明确唯一调用线程，除非接口明确声明线程安全。

### 1.2 通用结果

```cpp
enum class Status {
    ok,
    no_data,
    timeout,
    invalid_data,
    stale_data,
    queue_full,
    cancelled,
    fault,
};

struct ResultCode {
    Status status;
    std::uint32_t detail_code; // 稳定机器码；文本说明在慢路径格式化
};

enum class WaitResult { ready, timeout, cancelled, fault };

struct SourceDiagnostics {
    std::uint64_t received, delivered, dropped;
    std::uint64_t parse_errors, crc_errors, timestamp_errors;
    std::size_t queue_depth, queue_capacity;
};
struct CameraDiagnostics {
    std::uint64_t frames_received, frames_dropped, invalid_timestamps;
    std::size_t queue_depth, queue_capacity;
    double measured_fps;
};
struct CommandDiagnostics {
    std::uint64_t submitted, replaced, expired, sent, send_failures;
};
struct SinkDiagnostics {
    std::uint64_t accepted, dropped, failures;
    std::size_t queue_depth, queue_capacity;
    bool healthy;
};
```

`no_data/invalid/stale` 不等于 `fault`；队列满必须可计数。实时函数不通过动态字符串传错。

## 2. 时间戳契约

```cpp
enum class ClockDomain { unknown, monotonic, realtime, device };
enum class TimestampSource {
    unknown,
    camera_sdk_capture,
    host_callback_receive,
    imu_uart_receive_estimate,
    hardware_sync,
};
enum class TimestampQuality { unknown, estimated, calibrated, hardware_synchronized };

struct SensorTime {
    std::int64_t monotonic_ns;   // runtime 唯一比较/插值/deadline 轴；无效为 -1
    std::int64_t source_time_ns; // SDK/设备原值；无值时为 -1
    std::int64_t uncertainty_ns; // 未知时为 -1，不以 0 伪装精确
    ClockDomain source_domain;
    TimestampSource source;
    TimestampQuality quality;
};

struct RawSdkTime {
    std::int64_t seconds;
    std::int64_t microseconds;
    ClockDomain configured_domain;
};
struct TimeConversion {
    bool valid;
    SensorTime time;
    std::uint32_t error_code;
};
struct SyncDiagnostics {
    TimestampQuality quality;
    ClockDomain camera_domain;
    std::int64_t camera_offset_ns;
    std::int64_t camera_bias_ns;
    std::int64_t imu_latency_compensation_ns;
    std::int64_t estimated_uncertainty_ns;
    std::uint64_t rejected_timestamp_count;
};
```

规则：

1. 运行时比较、历史查询、滤波 `dt` 和 deadline 全用 `monotonic_ns`。
2. `CLOCK_REALTIME` 仅作为日志/会话时间；格式化事件可带 `wall_time_ns`，不得用于控制耗时。
3. SDK `Frame_Time` 原值必须保留；时钟域映射由 `ITimeSyncProvider` 完成并报告质量。
4. IMU 协议冻结且无 MCU 时间/样本号，IMU `monotonic_ns` 是接收时间减实测延迟后的估值，必须标为 `estimated`，不得伪装成 MCU 采样时间。
5. 队列积压、串口字节丢失、时间回退、时钟域不明都会降低时间质量或令样本不可用；不得无限外推或使用陈旧样本。

## 3. 坐标、向量与单位

```cpp
struct NavigationFrameTag {};
struct BallisticFrameTag {};
struct DartFrameTag {};
struct CameraFrameTag {};

template<class Frame>
struct Vec3 {
    double x;
    double y;
    double z;
};

template<class ToFrame, class FromFrame>
struct Rotation3 {
    double m[3][3]; // R_To_from_From，列向量：v_To = R * v_From
};

using AccelerationDartMps2 = Vec3<DartFrameTag>;
using OverloadDartG = Vec3<DartFrameTag>; // 每分量以 g 为单位的无量纲量
```

已确认坐标约定：

- Dart `D`：`+X` 前、`+Y` 上、`+Z` 右。
- Navigation `N`：原点发射点，`+X` 沿初始发射架方向的水平投影，`+Y` 上、`+Z` 右；初始发射倾角体现在 `R_N_from_D`，不倾斜导航系。
- Ballistic `B`：`+X` 沿速度；小攻角模型令 `D` 与 `B` 只差绕纵轴的 roll。
- Camera `C`：接口保留 `R_D_from_C`；默认单位矩阵，未来标定值只在一处注入。
- roll 单位 rad，正方向绕 `+X` 右手规则。
- LOS q 单位 rad，LOS rate 单位 rad/s，加速度/比力 m/s²，过载 g。
- `DirectionDartUnit` 为镖体系单位 LOS，无量纲；`DirectionRateDartPerSec` 是 LOS 惯性导数的镖体系表达，单位 1/s。惯性导数不是相机测得的相对导数，也不是导航系坐标分量。
- 指令过载是横向比力/g，包含抵消横向重力所需的分量，不是净运动学加速度/g。

任何 `Vec3` 跨模块传递必须通过坐标标签/字段名表达坐标系。禁止把“navigation_acceleration”“body_overload”等不同含义复用同一个无标签数组。

## 4. 领域数据

```cpp
enum class PixelFormat { bgr8, rgb8, yuyv, mjpeg };
struct PixelPoint { double u; double v; };
struct PixelRect { int x; int y; int width; int height; };
struct CameraConfig {
    std::uint32_t device_id;
    int width, height, fps;
    PixelFormat pixel_format;
    bool hardware_trigger;
};
enum class ObservationReason { found, not_found, invalid_frame, stale_frame };
enum class LaunchReason { threshold_confirmed, not_started, invalid_imu, stale_imu };
enum class GuidanceInvalidReason {
    none, launch_not_confirmed, stale_input, no_target, invalid_los,
    invalid_rate, invalid_attitude, algorithm_rejected
};

struct ImageView {
    const std::uint8_t* data;
    int width;
    int height;
    std::size_t stride_bytes;
    PixelFormat format;
};

struct PixelStorage; // opaque, pool-backed immutable storage
struct ImageBuffer {
    std::shared_ptr<const PixelStorage> owner;
    ImageView pixels; // view 必须在 owner 生命周期内有效
};

struct CameraFrame {
    std::uint64_t host_frame_id; // 主机诊断序号，不代表相机硬件序号
    SensorTime capture_time;
    std::int64_t callback_receive_monotonic_ns;
    ImageBuffer image;
};

struct ImuSample {
    std::uint64_t host_receive_chunk_id; // 诊断信息；不是协议中的 MCU 样本序号
    SensorTime sample_time;
    std::int64_t first_byte_receive_monotonic_ns;
    Vec3<DartFrameTag> acceleration_mps2;
    Vec3<DartFrameTag> angular_velocity_rad_s;
    double servo_p_normalized;
    double servo_y_normalized;
};

struct TargetObservation {
    std::uint64_t host_frame_id;
    SensorTime capture_time;
    bool valid;
    PixelPoint center; // 图像坐标：u 向右、v 向下
    PixelRect bounds;
    bool confidence_valid;
    double confidence;
    ObservationReason reason;
};

struct AttitudeState {
    SensorTime time;
    Rotation3<NavigationFrameTag, DartFrameTag> navigation_from_dart;
    double pitch_rad;
    double roll_rad;
    bool valid;
};

template<class Frame>
struct LosMeasurement {
    SensorTime time;
    bool valid;
    double q_y_rad;
    double q_z_rad;
};

template<class Frame>
struct LosRate {
    SensorTime time;
    bool valid;
    double q_y_rad_s;
    double q_z_rad_s;
};

enum class LaunchPhase { unknown, pre_launch, launched };
struct LaunchState {
    LaunchPhase phase;
    SensorTime transition_time;
    std::uint64_t generation;
    LaunchReason reason;
};

struct FlightStateSnapshot {
    AttitudeState attitude;
    LaunchState launch;
};

struct ImuSampleAtFrame {
    SensorTime time;
    Vec3<DartFrameTag> angular_velocity_rad_s;
    bool valid;
};

struct VectorLosEstimate {
    bool valid;
    DirectionDartUnit direction;
    DirectionRateDartPerSec inertial_direction_rate;
};

struct LosRateFilterInput {
    LineOfSight line_of_sight;
    AttitudeState attitude;
    double dt_seconds;
    std::uint64_t generation;
};

struct LosRateEstimate {
    bool valid; // 制导估计的共同有效性，不等同于角度诊断有效性
    LineOfSightAngularVelocity angular_velocity;
    VectorLosEstimate vector;
};

struct GuidanceInput {
    std::uint64_t source_frame_id;
    SensorTime time;
    LineOfSight line_of_sight;
    LineOfSightAngularVelocity angular_velocity;
    Quantity<MetersPerSecond> speed_mps;
    AttitudeState attitude;
    VectorLosEstimate vector_los;
};

struct GuidanceDiagnostics {
    AccelerationBallisticMps2 navigation_acceleration; // 历史名，弹道系指令比力
    AccelerationDartMps2 body_acceleration; // 镖体系指令比力，含重力补偿并限幅
    DartOverloadG body_overload;
    Quantity<GravityG> command_overload;
    Quantity<Radian> command_phase; // 弹道系 Y/Z 指令相位
};

struct GuidanceCommand {
    std::uint64_t generation;
    std::uint64_t source_frame_id;
    SensorTime source_time;
    bool valid;
    GuidanceInvalidReason invalid_reason;
    DartOverloadG overload_g;
};

struct GuidanceOutput {
    bool valid;
    GuidanceInvalidReason invalid_reason;
    GuidanceCommand command;
    GuidanceDiagnostics diagnostics;
};
```

这些类型应为简单值类型，不依赖 UI、SDK 或日志格式；图像像素缓冲采用拥有型句柄，`ImageView` 只在 owner 生命周期内借用。

## 5. 输入和同步接口

```cpp
class IImuSource {
public:
    virtual ResultCode start() noexcept = 0;
    virtual bool try_pop(ImuSample* out) noexcept = 0;
    virtual WaitResult wait_for_data_for(std::chrono::nanoseconds timeout) noexcept = 0;
    virtual SourceDiagnostics diagnostics() const noexcept = 0;
    virtual void request_stop() noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual ~IImuSource() = default;
};

class ICameraSource {
public:
    virtual ResultCode configure(const CameraConfig&) noexcept = 0;
    virtual ResultCode start() noexcept = 0;
    virtual bool try_pop(CameraFrame* out) noexcept = 0;
    virtual WaitResult wait_for_frame_for(std::chrono::nanoseconds timeout) noexcept = 0;
    virtual CameraDiagnostics diagnostics() const noexcept = 0;
    virtual void request_stop() noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual ~ICameraSource() = default;
};

class ITimeSyncProvider {
public:
    virtual TimeConversion convert_camera_time(
        RawSdkTime sdk_time,
        std::int64_t receive_monotonic_ns) noexcept = 0;
    virtual SyncDiagnostics diagnostics() const noexcept = 0;
    virtual ~ITimeSyncProvider() = default;
};
```

契约：

- 回调入口尽早采样主机单调接收时间；只复制/取得拥有型帧缓冲并入有界队列，不做解码、识别、日志或串口操作。
- SDK capture time 与 callback receive time 是不同字段。SDK 时间域未知时不得悄悄回退成 capture time=receive time。
- 当前 `ImuSource` 适配器保留 20-byte 协议和 CRC/字段转换；协议 parser 不拥有 UART fd。
- IMU 200 Hz 表示配置/实测速率；由于线格式无 MCU 序号，统计中的 chunk id 不能当作样本号。
- 未来硬触发可新增 `ISyncEventSource`（触发 id、观测时间、边沿/质量）；本阶段不要求提供实际硬触发实现。触发必须能进入相机和下位机/IMU 时间模型，是否能在冻结 UART 外增加 GPIO/公共脉冲需要实机阶段确认。

## 6. 算法接口

```cpp
class IVisionDetector {
public:
    virtual ResultCode process(const CameraFrame&, TargetObservation*) noexcept = 0;
    virtual void reset(std::uint64_t generation) noexcept = 0;
    virtual ~IVisionDetector() = default;
};

class IAttitudeEstimator {
public:
    virtual ResultCode update(const ImuSample&, AttitudeState*) noexcept = 0;
    virtual void reset(std::uint64_t generation) noexcept = 0;
    virtual ~IAttitudeEstimator() = default;
};

class ILaunchDetector {
public:
    virtual ResultCode update(const ImuSample&, LaunchState*) noexcept = 0;
    virtual void reset(std::uint64_t generation) noexcept = 0;
    virtual ~ILaunchDetector() = default;
};

class IFlightStateEstimator {
public:
    virtual ResultCode update(const ImuSample&,
                              FlightStateSnapshot*) noexcept = 0;
    virtual void reset(std::uint64_t generation) noexcept = 0;
    virtual ~IFlightStateEstimator() = default;
};

class ILosEstimator {
public:
    virtual ResultCode estimate(const TargetObservation&,
                                LosMeasurement<DartFrameTag>*) noexcept = 0;
    virtual ~ILosEstimator() = default;
};

class ILosRateFilter {
public:
    virtual domain::LosRateEstimate update(const domain::LosRateFilterInput&) = 0;
    virtual domain::LineOfSightRateSample sample() const = 0;
    virtual void reset() noexcept = 0;
    virtual ~ILosRateFilter() = default;
};

class IGuidanceLaw {
public:
    virtual domain::GuidanceOutput calculate(const domain::GuidanceInput&) = 0;
    virtual ~IGuidanceLaw() = default;
};
```

契约：

- 识别只输出 observation，不更新全局状态、不提交命令。
- Attitude 与 Launch 作为不同概念供下游使用。运行时先依赖 `IFlightStateEstimator`，由现有 `AttitudeEstimator` 适配器一次更新并返回两者，避免重复消费 IMU；在行为回归通过后，内部实现才可组合独立 `IAttitudeEstimator` 和 `ILaunchDetector`。
- LOS/filter/guidance 的有状态实例固定由 FramePipeline 单线程调用；新起飞 generation、长丢帧或时间异常时显式 reset。
- `GuidanceCommand` 的过载坐标固定为 Dart；输出与串口编码分离。
- 对无效状态返回原因，不用“零向量”代表 invalid。由 Runtime 的基线兼容策略决定是否提交/不提交；不得由单个算法隐式发安全命令。

### 6.1 可选镖体系矢量 PN

`[guidance] guidance_algorithm=png|body_vector_pn` 控制工厂成对选择滤波器和制导律；缺省仍为原 PNG。原标量滤波实现不伪造矢量估计，新制导律也不从角速率诊断回退重建矢量状态。

`BodyVectorLosFilter` 保存六状态 `[l_D,u_D]` 和完整 6×6 协方差。测量为单位 LOS；`u_D` 为惯性 LOS 导数的镖体系表达。两帧的坐标运输采用：

```
C = transpose(R_N_from_D_current) * R_N_from_D_previous
l_pred = C * (l_previous + dt * u_previous)
u_pred = C * u_previous
```

完整协方差同样运输，测量更新采用 Joseph 形式；单位长度和切向约束通过归一化映射及其 Jacobian 同时施加到状态和协方差。姿态运输已包含 IMU 转动补偿，不能再叠加 `omega_body × l`。

首帧只建立基准，不传播占位间隔。丢目标、IMU/姿态匹配失败、时间异常、generation 切换后流水线清除连续段；新滤波器也在超过 `body_vector_los_maximum_gap_ms` 时以当前帧重建基准。恢复后的第一帧不输出有效速率。新算法不对丢失目标盲目外推制导。

新 PN 使用单个导航系数，速度方向按小攻角/侧滑模型近似为镖体 +X：

```
omega_LOS_D = l_D × u_D
a_PN_D = N * V * (omega_LOS_D × e_x)
g_D = transpose(R_N_from_D) * (0,-g,0)
f_command_D = a_PN_D - (I-e_x*transpose(e_x)) * g_D
n_command_D = f_command_D / g
```

导航项不再额外按 roll 旋转，也不使用旧横向 `cos_pitch` 因子。重力补偿后按总 Y/Z 比力同比例限幅，取过载上限和加速度上限/g 的较小值；最终 X=0、Y/Z 分别下发。弹道系诊断量仅为记录转换，不反馈到镖体系命令计算。

角速率展示由矢量投影得到，LOS 仰角极点处可以无效，但不影响有效矢量 PN。LOS 离轴角不要求很小，但速度轴近似、实际机动能力、闭环延迟仍限制适用范围，不能据此承诺命中。

新配置键：`body_vector_navigation_constant`、`body_vector_los_sigma_a`（1/s²）、`body_vector_los_sigma_l`（无量纲）、`body_vector_los_initial_rate_covariance`（(1/s)²）、`body_vector_los_maximum_gap_ms`、`body_vector_maximum_command_overload`（g）、`body_vector_maximum_command_acceleration`（m/s²，`inf` 可禁用）。重力共用现有 `png_gravity` 配置和姿态估计的 `gravity_mps2`，速度仍为 `launch_speed_mps`。

默认 N=3、sigma_a=8、sigma_l=0.0035、初始速率方差=1、最大间隔=200ms；默认总比力限制仍为 1g/9.80665m/s²，不擅自提高硬件上限。水平 1g 已全部用于抵消重力，没有额外净向上机动余量。

“滚转一致性”验证保持同一物理场景并同步转换输入；不同 roll 的镖体系输出应不同，但转换回相同惯性系后的指令一致。固定镖体系输入数值只改变 roll，不是这个等价性测试。

## 7. 控制和观测输出接口

制导命令仍由独立的 `CommandSink` 提交；观测输出使用 `RuntimeOutputPorts` 中的窄接口，帧输出消费不可变的 `ProcessedFrame`（包含共享图像所有权、时间戳和 `source_frame_id`）：

- `IVideoRecorder`：`start/reset/record/take_rearm_request/cancel/stop/last_error`。
- `ITelemetryRecorder`：与视频 recorder 相同的生命周期和逐帧记录接口。
- `IVisualizationSink`：`start/reset/publish/cancel/stop/last_error`。
- `IImuRecorder`：接收 `domain::ImuKinematicSample`，以非阻塞 `try_record` 提交，并提供 `SinkDiagnostics`。
- `IAsyncLogger`：接收固定大小的 `RuntimeLogEvent`，提供诊断并在停止时 flush。未注入 logger 或 logger 未运行时不输出日志；拒绝提交时也不回退到同步 I/O。启动错误和运行期故障由 `last_error()` 保留，App 决定如何展示。

`SinkDiagnostics` 记录 enabled/healthy、accepted/dropped/failure 计数、队列深度/容量和 sink 自身的 `last_error`。`RuntimeOutputDiagnostics` 分别命名 video、telemetry、visualization、IMU recording 和 logging；未启用 sink 的快照为 disabled、healthy、计数为零。

每个启用的帧 sink 由独立的有界 latest-frame 队列和 worker 服务。队列满时只丢弃该 sink 最旧的待处理帧；`FramePipeline` 仅执行有界队列 fan-out，不在制导/命令路径执行编码、文件或网络 I/O。图像像素通过共享所有权传递，不深拷贝。IMU recorder 和 logger 分别使用独立队列/worker；IMU 队列满仍按既有策略触发运行时故障，日志丢弃、写入或 flush 故障只更新日志诊断，不升级为制导故障。

视频、遥测或可视化 worker 失败仍报告既有 frame-output 故障并取消 runtime；IMU 记录提交失败仍报告运行时故障。该阶段不启用可选输出失败后继续制导的策略。命令 writer 与观测输出队列绝不共用，也不改变串口控制帧、CRC、量化或 body-y/body-z 到 pitch/yaw 的映射。

## 8. 运行时组装和输出模式

App 是唯一 Composition Root。`make_runtime_output_ports` 按既有 `output.mode` 装配：

| `output.mode` | 帧输出 | 其他输出 |
|---|---|---|
| `debug` | 连续本地视频 + 逐帧遥测 CSV | IMU CSV + 异步日志 |
| `match` | 起飞窗口本地视频 + 逐帧遥测 CSV | IMU CSV + 异步日志 |
| `rtsp` | RTSP 可视化 | IMU CSV + 异步日志 |

`RuntimeSupervisor` 持有输出端口并统一管理 start/cancel/close/join/stop 生命周期。`FramePipeline` 向每个帧输出 worker 独立提交同一 processed-frame snapshot；`ImuPipeline` 将 IMU 样本提交给独立 recorder。match 模式的视频与遥测 sink 共享路径及 generation/window 协调元数据，但保留独立队列、writer、错误状态；只有两者均成功完成窗口后才发出 rearm。各文件可能因独立队列丢帧而包含不同帧子集，遥测 CSV 保留原有列顺序并在末尾附加 `source_frame_id` 供关联。

输出诊断由 `RuntimeSupervisor::output_diagnostics()` 汇总，应用周期性及退出时显示。输出故障策略、命令生成/提交顺序、坐标换算和传感器协议保持不变。

### 8.1 当前源码边界

- `include/mosas/runtime/processed_frame.hpp`、`log_event.hpp`、`output_diagnostics.hpp` 分别定义帧快照、日志事件和诊断值；`output_ports.hpp` 定义窄输出接口和注入所有权集合。日志生产者不需要包含具体 logger 的头文件。
- `include/mosas/output/` 声明具体输出适配器，`src/output/` 实现录像、CSV、RTSP 渲染与发送。`RtspVisualizationSink` 保持首帧懒启动，不在 Factory 或 supervisor 中执行网络工作。
- App 通过公共 `output/recording_paths.hpp` 分配一次运行的录制路径；会话标识生成是 `src/output/recording_session_id.hpp` 声明的私有实现，禁止跨模块相对路径包含。
- `src/app/output_factory.cpp` 只做模式选择、配置转换和组件组装；`src/runtime/` 下的 pipeline、dispatcher 及协调器头文件继续保持私有。
- 此处输出接口仍属于现有 runtime 边界，使用 runtime 的相机帧类型；不将其仅改名搬入 `ports/`，以免引入 `ports → runtime` 的反向依赖。

## 9. 相关验收测试

- **接口与队列**：`output_contracts_test` 检查窄端口与 disabled-neutral 诊断；`multi_output_dispatcher_test` 验证阻塞 sink 不影响其他帧 worker、队列丢弃和错误按 sink 隔离。
- **运行时集成**：`runtime_output_isolation_test` 在视频 worker 被同步屏障阻塞期间，逐帧释放相机输入并验证 command submit 与遥测 worker 仍继续；`runtime_supervisor_lifecycle_test` 覆盖故障取消、rearm、停止和重启。
- **记录器和故障语义**：`local_recorders_test` 检查视频/CSV、窗口、部分停止及 rearm；`output_sink_diagnostics_test` 和 `imu_pipeline_test` 检查 IMU 队列故障传播以及日志丢弃/写入/flush 故障不升级为 runtime fault。
- **模式组合**：`mode_composition_test` 验证 debug/match/rtsp 组合、所有模式的 IMU recorder/logger、路径分配，以及 RTSP 适配器的懒启动/错误/重置；无线关闭时验证拒绝真实 RTSP 组装。
- **协议回归**：现有串口协议测试继续验证固定控制帧字节、CRC 与量化行为。

设置 `MOSAS_BUILD_APPLICATION=OFF`、`MOSAS_BUILD_OPENCV_ADAPTERS=ON`、`BUILD_TESTING=ON` 可构建本地输出及模式组合测试，无需配置 Nori SDK；`MOSAS_BUILD_WIRELESS` 控制是否构建 RTSP 适配器及链接 FFmpeg。纯 runtime 测试不需要 OpenCV。

这些 host 测试使用 fake sources/sinks，不依赖 Nori 硬件。A133P 实时性能、同步误差和 watchdog 行为仍须在目标板验收中测量，不从 host 测试推断。
