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
- 指令过载是下发的横向比力/g，不是净运动学加速度/g。PNG 与矢量 PN 均在 LOS 速率域扣除自然抛物线基准，不在输出端另加重力；矢量 PN 的基准方向经完整姿态转换到镖体系。

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
    Quantity<MetersPerSecond> speed_mps; // 与姿态同一时刻的纵向速度估计
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
    AccelerationDartMps2 body_acceleration; // 按所选制导律的补偿模型生成并限幅的镖体系指令比力
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
    bool launched; // 当前周期是否已按所选发射判据触发，独立于制导有效性
    bool target_found; // 当前帧视觉识别结果，独立于制导有效性
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
- Attitude 与 Launch 作为不同概念供下游使用。运行时先依赖 `IFlightStateEstimator`，由现有 `AttitudeEstimator` 适配器一次更新并返回姿态、起飞状态和纵向速度，避免重复消费 IMU；在行为回归通过后，内部实现才可组合独立 `IAttitudeEstimator` 和 `ILaunchDetector`。
- LOS/filter/guidance 的有状态实例固定由 FramePipeline 单线程调用；新起飞 generation、长丢帧或时间异常时显式 reset。所选发射判据触发前可识别目标和计算 LOS，但不更新飞行 LOS 速率滤波器、不计算制导或下发制导指令；触发后的首个有效图像仅建立新的速率基准。
- `GuidanceCommand` 的过载坐标固定为 Dart；输出与串口编码分离。
- 对无效状态返回原因，不用“零向量”代表 invalid。由 Runtime 的基线兼容策略决定是否提交/不提交；不得由单个算法隐式发安全命令。

### 6.1 同步纵向速度估计

`[guidance] launch_speed_mps` 的配置键保持不变，内部由 `AttitudeEstimatorConfig::launch_speed_mps` 持有，要求有限且大于零，表示发射触发确认帧采用的标定初速。触发前、确认帧和 reset 后保持该初值；确认帧不积分姿态或速度，下一连续有效 IMU 样本起，先传播姿态，再执行：

```
V_k = V_(k-1) + (ax_imu_k - gravity_mps2 * sin(pitch_k)) * dt
```

`ax_imu_k` 为输入的镖体 X 轴比力（m/s²），`pitch_k` 为当前帧俯仰角（rad），`dt` 使用实际相邻 IMU 时间戳差（s）。不回补所选判据的确认窗口，不额外扣除启动加速度基线。`acceleration` 从加速确认后的下一帧开始积分，仍可能包含剩余架上加速；`two_stage` 从离架确认后的下一帧开始积分。无效样本和非递增时间戳不更新输出状态；非递增样本忽略，无效样本中断积分连续性。超过 `attitude_maximum_gap_ms`（默认 20 ms）的断流区间不积分，恢复后从新的连续样本继续。

速度作为 `FlightStateSnapshot::speed_mps` 发布，随完整姿态写入同一历史槽并线性插值到相机曝光时刻；不外推、不跨起飞状态、起飞时刻或 generation 插值。PNG 和矢量 PN 均使用该曝光时刻速度，而非初值或最新 IMU 速度。积分结果不人为钳位为正数；速度非正时沿用两种制导律的无效输入处理，不回退为固定初速。

#### 6.1.1 可选发射触发判据和姿态积分边界

`[guidance] launch_detection_mode=acceleration|two_stage` 在启动时选择；未配置该键时仍为 `acceleration`，仓库示例配置显式选择 `two_stage`，未知值拒绝加载。估计器构造后模式固定，切换配置须重启；reset 保留模式，只清除标定、候选和触发锁存。此参数不改变重力补偿、LOS 算法或串口字节布局。

| 模式 | 触发时刻 | `launched` 的含义 |
| --- | --- | --- |
| `acceleration`（默认） | X 轴相对静止基线增量达到阈值，并连续满足确认时长 | 原单阶段加速判据确认，不保证实际离架 |
| `two_stage` | 加速确认后，再连续满足低比力离架判据 | 两阶段判据均确认 |

两种模式共用当前的静止标定、触发前非静止姿态冻结、异常保护及触发后积分。开关仅切换触发判据/时刻，不回滚旧版本的整套姿态传播或滤波实现。`LaunchPhase::launched`、串口发射标志和 match 录像窗口均跟随所选判据；`transition_time` 是该判据的确认帧时间，`LaunchReason::threshold_confirmed` 表示所选判据已满足。

1. 待加速阶段只使用满足静止条件（比力模长接近重力且角速度足够小）的样本校准姿态、陀螺零偏和 X 轴基线；未获得静止基线时禁止启动发射检测。其他触发前样本保持姿态，不进行陀螺传播。
2. 两种模式共用 `launch_ax_threshold_mps2`（默认 14.709975 m/s²）和 `launch_confirm_ms`（默认 20 ms）。加速确认后，`acceleration` 立即触发并锁存；`two_stage` 仅进入待离架状态，冻结校准结果但不启用制导。
3. 仅 `two_stage` 在待离架阶段使用 `launch_release_acceleration_max_mps2`（默认 3 m/s²）及 `launch_release_confirm_ms`（默认 10 ms），低比力连续确认后才触发。比力阈值必须严格低于静止校准区间，单独失重或短冲击不能绕过加速确认。
4. 仅 `two_stage` 使用 `launch_release_timeout_ms`（默认 500 ms）：加速确认后超时未确认离架，取消候选并要求重新静止校准；超时必须大于低比力确认时长，允许恰在边界完成确认。两种模式均在触发前遇到新的无效样本或超过 `attitude_maximum_gap_ms` 的断流时取消候选并要求重校准，不把不连续时长算作有效证据。重复和回退的非负时间戳优先忽略，即使载荷无效也不能清除当前检测证据或打断后续积分。

所有模式都校验完整配置，包括当前模式未使用的三项离架参数，避免切换后出现潜伏的无效配置。

这些默认值依据 2026-10-17 的单次 IMU 记录设定，必须通过更多实测确认；IMU 低比力是离架证据，不是独立硬件离架传感器。记录中的架上加速出现约 58.85 m/s² 的平台，可能存在量程饱和，因此不从该段积分反推离架速度。

### 6.2 默认 PNG 的抛物线 LOS 角速度补偿

默认 `png` 在角速度域扣除自然抛物线基准，不改变标量 LOS 滤波器的测量状态；`body_vector_pn` 使用同一基准的矢量形式（见 6.3）。PNG 先用当前完整姿态将镖体系 LOS 单位方向转到导航系，再求惯性 LOS 仰角：

```
l_N = R_N_from_D * l_D
q_y_inertial = atan2(l_N.y, hypot(l_N.x, l_N.z))
q_dot_y_parabola = -g0 * cos(q_y_inertial) / (2 * V_actual)
s_y = q_dot_y_measured - q_dot_y_parabola
n_y = N_y * V_actual * s_y / g0
```

`q_dot_y_measured` 使用已有 IMU 转动补偿、滤波后的惯性 LOS 角速度（rad/s），不额外积分成视线角；`V_actual` 使用曝光时刻同步的动态速度。完整姿态用于求惯性仰角，不能直接用机体视线角、机体俯仰或两者相加替代。

补偿发生在导引律增益之前；原纵向输出中的 `+cos(pitch)` 删除。若测量速率恰好等于自然抛物线速率且横向速率为零，过载输出为零，不再自动叠加保持高度的比力。横向 PNG 公式、roll 变换、合成过载限幅和加速度限幅保持原逻辑；姿态改为传递完整旋转矩阵，pitch/roll 从该矩阵求得，旧的独立角度接口移除。

`PngCalculationTrace` 额外记录惯性仰角、测量纵向速率、抛物线基准速率和补偿后速率；原始/滤波 LOS 观测及 overlay 不因补偿而被覆盖。速度或姿态无效时拒绝计算，不使用固定初速或未补偿旧公式回退。

### 6.3 可选镖体系矢量 PN

`[guidance] guidance_algorithm=png|body_vector_pn` 控制工厂成对选择滤波器和制导律；未配置该键时仍使用上述带抛物线速率补偿的 PNG。仓库 `config/mosas_guidance.conf` 显式选择 `body_vector_pn`。标量滤波实现不伪造矢量估计，新制导律也不从角速率诊断回退重建矢量状态。

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
l_N = R_N_from_D * l_D
e_up_N = (0,1,0)
u_parabola_N = -g / (2 * V_actual) * (e_up_N - l_N.y * l_N)
u_parabola_D = transpose(R_N_from_D) * u_parabola_N
u_compensated_D = u_D - u_parabola_D
omega_compensated_D = l_D × u_compensated_D
f_command_D = N * V_actual * (omega_compensated_D × e_x)
n_command_D = f_command_D / g
```

`u_parabola_N` 是 `q_dot_y_parabola=-g*cos(q_y_inertial)/(2*V_actual)`、惯性 LOS 方位角速率为零的单位方向导数：`e_up_N-l_N.y*l_N` 的长度为 `cos(q_y_inertial)`，不需要除以该余弦，故导航系仰角极点也连续有效。基准经完整姿态转换到镖体系后才扣除，roll=90° 时补偿不会仍固定在镖体 Y。滤波器原始/滤波方向与惯性导数保持不变，仅在制导计算局部使用补偿值；不使用标量速率诊断作为回退。

导航项不再额外按 roll 旋转，也不使用旧横向 `cos_pitch` 因子；输出端的 `+g*R[1][1/2]` 横向重力项已删除，避免重复补偿。最终按总 Y/Z 指令比力同比例限幅，取过载上限和加速度上限/g 的较小值；X=0、Y/Z 分别下发。弹道系诊断量仅为记录转换，不反馈到镖体系命令计算。

角速率展示由矢量投影得到，LOS 仰角极点处可以无效，但不影响有效矢量 PN。LOS 离轴角不要求很小，但速度轴近似、实际机动能力、闭环延迟仍限制适用范围，不能据此承诺命中。

新配置键：`body_vector_navigation_constant`、`body_vector_los_sigma_a`（1/s²）、`body_vector_los_sigma_l`（无量纲）、`body_vector_los_initial_rate_covariance`（(1/s)²）、`body_vector_los_maximum_gap_ms`、`body_vector_maximum_command_overload`（g）、`body_vector_maximum_command_acceleration`（m/s²，`inf` 可禁用）。重力共用现有 `png_gravity` 配置和姿态估计的 `gravity_mps2`；`launch_speed_mps` 仅为标定发射速度初值，不再作为固定制导速度。

C++ 参数结构的缺省值保持 N=3、sigma_a=8、sigma_l=0.0035、初始速率方差=1、最大间隔=200ms，总比力限制为 1g/9.80665m/s²。仓库配置显式采用离线回放候选 sigma_a=4、sigma_l=0.0005，初始速率方差和最大间隔保持不变；总比力上限显式设为 0.3g、加速度上限保持 9.80665m/s²，避免算法切换意外提高既有上限。候选参数尚需独立实测验证。限幅作用于抛物线速率补偿后的最终指令，不单独保留保持高度的重力前馈。测得的矢量惯性导数等于自然抛物线基准时输出为零；零 LOS 速率并不等于基准速率，例如水平前向 LOS、零速率时未限幅指令为 `N/2 g`。矢量 PN 保留自身几何投影，不承诺离轴场景中与标量 PNG 的各轴输出完全相同。

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

视频、遥测或可视化 worker 失败仍报告既有 frame-output 故障并取消 runtime；IMU 记录提交失败仍报告运行时故障。该阶段不启用可选输出失败后继续制导的策略。命令 writer 与观测输出队列绝不共用。

串口协议编码适配器将 `GuidanceCommand` 编码为与 `main` 一致的 14-byte 下行帧，不执行 PNG 或坐标变换。body-y/body-z 分别映射 pitch/yaw，按 0.01 g 量化为小端 int16；制导输出无效时 pitch/yaw 编码为零，识别位独立取最近提交命令的视觉结果，命令过期时归零。

`SerialCommandSink` 从发送线程启动起按 `[command].send_rate_hz` 周期发送，默认 100 Hz（10 ms），允许 1～1000 Hz；图像采集目标仍为 120 FPS，实际制导计算频率由图像处理决定。

- 生产者使用现有 latest-value mailbox 发布最新结果，发送线程仅采样最新值，不积压 FIFO、不插值。
- 结果更新较慢时重复最近值，但 `[command].command_timeout_ms`（默认 100 ms，必须大于 0）从新命令提交的单调时刻计算，既不从首次消费时刻计算，也不因重发而续期；来源图像时间和 frame ID 不变。该超时检测命令生产断流，不替代曝光时间、IMU 匹配和输入有效性检查。
- 超时后清零过载及识别位，保留最近命令的发射标志；未发射时始终发送零过载。新代际及重启清除缓存，收到新命令前发送零过载及零状态标志。
- 取消或停止会唤醒发送线程并停止周期发送；串口写失败计数保留，调度或写入延迟只跳过错过的周期，不突发补发。

100 Hz 是调度目标而非硬实时保证，断线、进程退出后的执行端行为仍须由下位机 watchdog 保证。`log` 输出及仿真命令 sink 仍按新结果处理，不做周期重发。

| 字节位置（从 0 开始） | 含义 |
| --- | --- |
| 0..1 | 帧头 `AA 55` |
| 2 | 发射标志：所选发射判据未确认为 `0`，确认后为 `1`（原 Roll 字节） |
| 3 | 识别位：最近提交的有效期内结果识别到目标为 `1`，未识别、未提交或超时为 `0` |
| 4..5 | Yaw 过载 |
| 6..7 | Pitch 过载 |
| 8..9 | 保留，固定 `0` |
| 10..11 | CRC-16/CCITT-FALSE，小端，覆盖 bytes 2..9（含发射标志和识别位） |
| 12..13 | 帧尾 `0D 0A` |

发射标志采用 `launch_detection_mode` 所选判据：`acceleration` 加速确认即置 `1`，`two_stage` 还须离架确认；触发后本周期保持为 `1`，复位后为 `0`。目标丢失、制导失效或故障清零均不清除已触发状态。触发后的首个有效图像仅建立速率基准，即使识别成功也发送识别位 `1` 和零过载；后续目标丢失时识别位恢复为 `0`。触发前不提交制导命令，但串口发送线程仍周期发送发射标志 `0` 和零过载的控制帧；曝光时刻早于确认的历史图像，包括插值区间跨触发边界的图像，不得借用最新 IMU 的已触发状态更新命令。周期帧的发射标志来自最近提交的当前代际命令，不因没有新图像而自行推断 IMU 状态。运行时故障清零时识别位为 `0`。下位机须按此布局解析，不再将 bytes 2..3 解释为双字节 Roll 过载。20-byte IMU 上行帧及字段转换不变。

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
- **发射触发**：`launch_detection_test`、`attitude_adapter_test` 验证默认加速触发、可选两阶段确认、误触发拒绝、超时边界、断流/异常样本及复位重校准；`body_vector_runtime_test` 在两种模式及两种 PN 下验证触发前不建立飞行滤波状态、跨触发边界不下发、使用触发后的动态速度；默认算法和仿真测试覆盖参数解析、缺省/非法模式、确认时刻元数据及两种模式下真实 Socket 录制链路。
- **协议回归**：串口协议测试验证固定帧长、过载量化、两个标志独立为 0/1 且都参与 CRC；命令适配器和运行时测试验证正常制导、首帧速率不可用、目标丢失及故障清零的状态标志，并保持所选判据触发前不提交制导命令。`serial_command_sink_test` 使用 Linux 伪终端验证启动零帧、120 Hz 结果按 100 Hz 采样、慢更新时重发、按提交时刻超时、失效归零、代际隔离、重启和可中断停止；触发前的周期串口帧始终为零过载。

设置 `MOSAS_BUILD_APPLICATION=OFF`、`MOSAS_BUILD_OPENCV_ADAPTERS=ON`、`BUILD_TESTING=ON` 可构建本地输出及模式组合测试，无需配置 Nori SDK；`MOSAS_BUILD_WIRELESS` 控制是否构建 RTSP 适配器及链接 FFmpeg。纯 runtime 测试不需要 OpenCV。

这些 host 测试使用 fake sources/sinks，不依赖 Nori 硬件。A133P 实时性能、同步误差和 watchdog 行为仍须在目标板验收中测量，不从 host 测试推断。

## 10. 可选 Gazebo Socket 仿真边界

`MOSAS_BUILD_SIMULATION_APPLICATION=ON` 构建 `mosas_simulation_app`，要求 OpenCV adapters 和已安装的 `dart_sim_transport::dart_sim_transport`，不要求 Nori、ROS 或 Gazebo。原 `config/mosas_guidance.conf` 是相机/IMU、算法、起飞/速度及输出策略的唯一来源，通过同一 `AppConfigLoader` 加载且只读；硬件设备、曝光和时间戳 bias 不用于仿真。

`config/mosas_simulation.conf` 仅接受 `[simulation]`：必填 `app_config`、`recording_directory`，可选 `socket_directory`、`sync_budget_ms`。相对路径按包装 INI 目录解析。原 recording_path 的基础文件名保留，只重定向父目录；旧私有 `[guidance]`/`[output]`、rtsp、非 free_run 和不可表示的 K 明确拒绝。ROS 管理器/CLI 的配置引用须与 app_config 一致，更改原配置后两侧重启并重装模型。

- `SimulationSession` 拥有双 Socket 和独立接收线程；IMU 有界 FIFO、图像 latest-only。不可恢复的失图、缺样/溢出、回退或断开使该 session 失败，不在同一 session 重启算法。
- `SimulationImuSource`/`SimulationCameraSource` 实现原 runtime ports；RGB8 拥有型帧只转 BGR，不更改几何。Gazebo (+X 前,+Y 左,+Z 上) 转 Dart 为 `(x,z,-y)`，安装旋转为 `A*R*Aᵀ`。
- `SimulationClock` 仅返回 session 仿真时间。采集元数据使用 `ClockDomain::simulation`、`TimestampSource::simulation`、`TimestampQuality::simulation`；`monotonic_ns` 在此 session 内是仿真 ns，不是主机单调时钟。
- 相机等待 `latest_state()` 的已处理水位，默认预算为 10 ms 仿真时间。预算不足丢帧；主机短时等待超时保留待处理帧。水位不代替原历史插值和 generation 验证。
- `SimulationCommandSink` 将 `left_g=-overload_g.z`、`up_g=overload_g.y`，保留来源图像时间及 frame ID。Socket session 与 MOSAS generation 分别检查；generation 切换清除未发送旧命令。
- `validate_simulation_metadata` 按已加载原配置核对握手尺寸/RGB8、相机/IMU rate、fx/fy/cx/cy、安装旋转、世界重力和 g0；保持 1e-6 相对/绝对容差。实际渲染须使用原非中心 K，Gazebo 侧恢复已知 XML 序列化精度后校验完整投影才报告 camera_ready，不只修改握手标定。
- `SimulationRunner` 为每个 ready 新 session 创建 RuntimeSupervisor，复用原算法、输出工厂并注入仿真 clock。启动/请求速度使用原配置，不从物理真值更新；不匹配则拒绝。配置的输出 sink 在报告 ready 前及运行中须健康；这是仿真应用层策略，不改变硬件 Runtime 的非致命 IMU/logger 故障策略。
- `SimulationRunner::timing()` / `output_diagnostics()` 只读封装原诊断。finish 先 stop/join、编码/刷新，再报告各 sink 的 accepted/dropped/failures/last_error，最后清水位并销毁 session。配置 Hz 与主机实测吞吐分别报告；SimulationCameraSource 不把握手标称 Hz 冒充实际 capture FPS。
- 原 debug/match 输出使用独立运行目录，保存 AVI、遥测 CSV 和全程 IMU CSV。match 由原起飞判据自然触发，未满窗口正常停止也保存部分数据；下一 session 不覆盖上一发。当前 640×480 / 120 Hz、match 4000 ms 的 BGR 窗口约 442 MB，内存上限仍为 1,105,920,000 bytes，超限启动前拒绝；默认父目录 `/home/panshenghe/dart_sim_ws/recordings/`。

`simulation_config_test`、`simulation_contract_test` 验证原配置与握手；`simulation_sources_test`、`simulation_command_sink_test`、`simulation_sync_test`、`simulation_runner_test` 覆盖适配器、丢流/溢出、同步屏障、原插值拒绝及生命周期。`simulation_recording_test` 使用真实 Socket 来源、原算法和原输出，验证两发 match 部分落盘、完整 IMU、debug/prelaunch、文件隔离及错误可见，临时文件由 RAII 清理。这些不构成 Gazebo 闭环验收。

实际步骤、DSIM v1 和验收状态见 SIM 仓库 `docs/mosas-simulation.md`。2026-10-14 的实际尝试受无可用 X/Ogre 渲染及 DDS 服务未 ready 阻塞，没有发射/实际录制；历史 PNG 非零执行证据采用旧参数，不能替代当前配置验收。
