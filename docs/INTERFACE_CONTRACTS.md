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
- LOS q 单位 rad，LOS rate 单位 rad/s，加速度 m/s²，过载 g。

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

struct GuidanceInput {
    std::uint64_t source_frame_id;
    SensorTime time;
    LosMeasurement<DartFrameTag> los;
    LosRate<DartFrameTag> los_rate;
    AttitudeState attitude;
    LaunchState launch;
    double speed_mps;
};

struct GuidanceResult {
    bool valid;
    GuidanceInvalidReason invalid_reason;
    OverloadDartG commanded_overload;
    Vec3<BallisticFrameTag> ballistic_acceleration_mps2;
    Vec3<DartFrameTag> dart_acceleration_mps2;
    double combined_overload_g;
    double command_phase_rad;
};

struct GuidanceCommand {
    std::uint64_t generation;
    std::uint64_t source_frame_id;
    SensorTime source_time;
    bool valid;
    GuidanceInvalidReason invalid_reason;
    OverloadDartG overload;
    bool launched; // 当前周期是否已检测到发射，独立于制导有效性
    bool target_found; // 当前帧视觉识别结果，独立于制导有效性
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
    virtual ResultCode update(const LosMeasurement<DartFrameTag>&,
                              const ImuSampleAtFrame&,
                              LosRate<DartFrameTag>*) noexcept = 0;
    virtual void reset(std::uint64_t generation) noexcept = 0;
    virtual ~ILosRateFilter() = default;
};

class IGuidanceLaw {
public:
    virtual ResultCode calculate(const GuidanceInput&,
                                 GuidanceResult*) noexcept = 0;
    virtual void reset(std::uint64_t generation) noexcept = 0;
    virtual ~IGuidanceLaw() = default;
};
```

契约：

- 识别只输出 observation，不更新全局状态、不提交命令。
- Attitude 与 Launch 作为不同概念供下游使用。运行时先依赖 `IFlightStateEstimator`，由现有 `AttitudeEstimator` 适配器一次更新并返回两者，避免重复消费 IMU；在行为回归通过后，内部实现才可组合独立 `IAttitudeEstimator` 和 `ILaunchDetector`。
- LOS/filter/guidance 的有状态实例固定由 FramePipeline 单线程调用；新起飞 generation、长丢帧或时间异常时显式 reset。
- `GuidanceCommand` 的过载坐标固定为 Dart；输出与串口编码分离。
- 对无效状态返回原因，不用“零向量”代表 invalid。由 Runtime 的基线兼容策略决定是否提交/不提交；不得由单个算法隐式发安全命令。

## 7. 控制和观测输出接口

```cpp
class ICommandSink {
public:
    virtual ResultCode start() noexcept = 0; // supervisor thread
    virtual bool try_submit(const GuidanceCommand&) noexcept = 0;
    virtual CommandDiagnostics diagnostics() const noexcept = 0;
    virtual void set_generation(std::uint64_t) noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual ~ICommandSink() = default;
};

struct ProcessedObservation {
    CameraFrame frame;
    TargetObservation target;
    AttitudeState attitude;
    LaunchState launch;
    LosMeasurement<DartFrameTag> los;
    LosRate<DartFrameTag> los_rate;
    GuidanceResult guidance;
    GuidanceCommand command;
};

class IImuRecorder {
public:
    virtual bool try_record(const ImuSample&) noexcept = 0;
    virtual SinkDiagnostics diagnostics() const noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual ~IImuRecorder() = default;
};

class IVideoRecorder {
public:
    virtual bool try_record(const ProcessedObservation&) noexcept = 0;
    virtual SinkDiagnostics diagnostics() const noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual ~IVideoRecorder() = default;
};

class ITelemetryRecorder {
public:
    virtual bool try_record(const ProcessedObservation&) noexcept = 0;
    virtual SinkDiagnostics diagnostics() const noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual ~ITelemetryRecorder() = default;
};

class IVisualizationSink {
public:
    virtual bool try_publish(const ProcessedObservation&) noexcept = 0;
    virtual SinkDiagnostics diagnostics() const noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual ~IVisualizationSink() = default;
};

enum class LogSeverity { debug, info, warning, error };
struct LogEvent {
    SensorTime event_time;
    std::int64_t wall_time_ns; // 仅供显示；实时线程不格式化字符串
    LogSeverity severity;
    std::uint32_t event_code;
    std::array<std::int64_t, 4> values;
};
struct LoggerDiagnostics : SinkDiagnostics {
    std::uint64_t flush_failures;
};

class IAsyncLogger {
public:
    virtual bool try_submit(const LogEvent&) noexcept = 0;
    virtual LoggerDiagnostics diagnostics() const noexcept = 0;
    virtual void stop_and_flush() noexcept = 0;
    virtual ~IAsyncLogger() = default;
};
```

生产组合使用 `IImuRecorder`、`IVideoRecorder`、`ITelemetryRecorder` 和 `IVisualizationSink` 等窄接口，各自具有独立队列/worker/错误状态。命令 writer 的队列与观测输出绝不共用。

当前协议编码适配器将 `GuidanceCommand` 编码为 14-byte 下行帧，不执行 PNG 或坐标变换。body-y/body-z 分别映射 pitch/yaw，按 0.01 g 量化为小端 int16；制导输出无效时 pitch/yaw 编码为零，识别位独立取当前帧视觉结果。

| 字节位置（从 0 开始） | 含义 |
| --- | --- |
| 0..1 | 帧头 `AA 55` |
| 2 | 发射标志：未发射为 `0`，已检测到发射为 `1`（原 Roll 字节） |
| 3 | 识别位：当前帧识别到目标为 `1`，未识别或未执行识别为 `0` |
| 4..5 | Yaw 过载 |
| 6..7 | Pitch 过载 |
| 8..9 | 保留，固定 `0` |
| 10..11 | CRC-16/CCITT-FALSE，小端，覆盖 bytes 2..9（含发射标志和识别位） |
| 12..13 | 帧尾 `0D 0A` |

发射标志由现有起飞检测提供，检测到发射后本周期保持为 `1`，复位后为 `0`；目标丢失、制导失效或故障清零均不清除已发射状态。识别成功但首帧视线角速度尚不可用时，仍发送识别位 `1` 和零过载；后续目标丢失时识别位恢复为 `0`。起飞前不发送控制指令的规则不变。下位机须按此布局解析，不再将 bytes 2..3 解释为双字节 Roll 过载。

## 8. 运行时组装接口

App 是唯一 Composition Root：

```cpp
struct RuntimeComponents {
    std::unique_ptr<ICameraSource> camera;
    std::unique_ptr<IImuSource> imu;
    std::unique_ptr<ITimeSyncProvider> time_sync;
    std::unique_ptr<IVisionDetector> vision;
    std::unique_ptr<IFlightStateEstimator> flight_state;
    std::unique_ptr<ILosEstimator> los;
    std::unique_ptr<ILosRateFilter> los_filter;
    std::unique_ptr<IGuidanceLaw> guidance;
    std::unique_ptr<ICommandSink> command;
    std::vector<std::unique_ptr<IImuRecorder>> imu_recorders;
    std::vector<std::unique_ptr<IVideoRecorder>> video_recorders;
    std::vector<std::unique_ptr<ITelemetryRecorder>> telemetry_recorders;
    std::vector<std::unique_ptr<IVisualizationSink>> visualizers;
    std::unique_ptr<IAsyncLogger> logger;
};
```

`RuntimeSupervisor` 拥有以上组件和 worker 生命周期；`ImuPipeline` 维护 IMU/姿态历史；`FramePipeline` 对单帧执行时间匹配和算法；`OutputDispatcher` 将只读 processed event 分送到独立可选 sinks。配置只选择实现和参数，不把设备、算法、UI 细节传进其他模块。

## 9. 验收接口测试

- **Domain tests**：帧变换方向、D/B/N 坐标、roll 正负、单位换算、时间域映射、过载量化。
- **Protocol tests**：固定 IMU/下行帧长度、字节序、CRC、饱和/非法数值；下行帧验证发射标志和识别位独立为 0/1 且均参与 CRC，未授权的重构不得改变协议字节输出。
- **Algorithm contract tests**：相同输入/时间戳下旧视觉、姿态/起飞、LOS filter、PNG 实现与基准输出一致。
- **Port contract tests**：Fake camera/IMU 的顺序、取消、timeout、队列满和诊断计数；CommandSink latest-wins / generation / stale behavior。
- **Pipeline integration tests**：虚拟单调时钟 + prerecorded camera/IMU + fake command/outputs；确保 recorder/logger 阻塞或满队列不阻塞 control submit。
- **Target acceptance**：A133P 640×480@120 FPS + IMU 200 Hz；记录 capture→processing→command submit→UART write 完成的 P50/P95/P99/max，队列延迟/丢弃、IMU 对齐误差和输出故障恢复。

数值误差限、传感器时间对齐误差、watchdog 和下位机失联行为属于目标板验收前必须补齐的参数，不在接口草案中伪造默认值。
