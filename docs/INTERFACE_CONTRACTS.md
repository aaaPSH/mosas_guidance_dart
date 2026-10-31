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
    sequence_fixed_period,
    hardware_sync,
    simulation,
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
4. 相机与串口 IMU 在硬件源中采用固定序号时间轴（`sequence_fixed_period`），根据首帧同步基线及标定采样频率计算 `T(n) = T0 + round((n - n0) * 1e9 / f)`。SDK Frame_Time 与串口接收时间保留作为原始诊断证据，不逐帧替换采样时间。
5. 启动时通过稳定观察窗口建立基线与同步屏障；发射前发生未知失步可重新同步并重置跨时间段状态；发射后时间轴基线与周期锁定，时间异常只记入诊断，不触发重锚或清零，既有陈旧输入与 I/O 保护仍由其自身机制判定。
6. 队列积压、串口未知失步、时钟域不明都会降低时间质量或使样本不可用；不根据接收时间猜测补号。
7. 相机固定序号时钟的候选基线、接收观察、当前及启动时间均使用同一个 bias 平移，禁止仅补偿基线或仅补偿接收时间。`sensor_time.source_time_ns` 保留未补偿的 SDK 单调转换值，帧的 `received_timestamp_ns` 保留原始主机接收时间。固定补偿不进入失步残差、不改变预热时长；原有未来／过旧保护仍使用原始接收时间。负时间和有符号溢出必须拒绝，不做饱和修补。

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
- `DartLineOfSight::direction` 使用无量纲的镖体系单位向量 `DirectionDartUnit`，不经过镖体系视线角。滤波器使用曝光姿态直接旋转到导航系，输出 `DirectionNavigationUnit` 和 `DirectionRateNavigationPerSec`；导数单位为 1/s，不是相机测得的相对导数。
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

struct DartLineOfSight {
    bool valid;
    DirectionDartUnit direction; // 相机光心指向目标，有限且单位化
};

struct LineOfSightAngularVelocity {
    bool valid;
    Quantity<RadiansPerSecond> q_y;
    Quantity<RadiansPerSecond> q_z; // 横向有效角速率 cos(q_y) * dq_z/dt
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
    DirectionNavigationUnit direction;
    DirectionRateNavigationPerSec inertial_direction_rate;
};

struct LosRateFilterInput {
    DartLineOfSight line_of_sight;
    AttitudeState attitude;
    double dt_seconds;
    std::uint64_t generation;
    Quantity<MetersPerSecond> speed_mps; // 曝光同步纵向速度；启用杆臂补偿时须有限且非负
};

struct LosRateEstimate {
    bool valid; // 惯性角度及角速率状态有效；角度滤波在方位极点拒绝输出
    LineOfSightAngularVelocity angular_velocity;
    VectorLosEstimate vector;
};

struct LineOfSightRateSample {
    DartLineOfSight line_of_sight;
    LineOfSightAngularVelocity raw_angular_velocity;
    LineOfSightAngularVelocity filtered_angular_velocity;
    bool lever_arm_distance_valid; // 与角速率有效位独立；首帧距离可有效
    double lever_arm_distance_m;   // 本帧实际用于杆臂补偿的距离，m
};

struct GuidanceInput {
    std::uint64_t source_frame_id;
    SensorTime time;
    Quantity<MetersPerSecond> speed_mps;
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
    virtual domain::DartLineOfSight estimate(
        const domain::VisionDetection& observation,
        const domain::CameraIntrinsics& intrinsics,
        const domain::Rotation3<domain::DartFrameTag,
                                domain::CameraFrameTag>& dart_from_camera) = 0;
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

1. 待加速阶段要求比力模长接近重力、原始角速度模长不超过 `attitude_gyro_stationary_threshold_rad_s`（默认 0.05 rad/s），并连续满足 `attitude_stationary_confirm_ms`（默认且至少 200 ms）。确认前首次姿态无效且禁止启动发射检测；确认后用该连续段的重力矢量 EMA（`attitude_accel_filter_alpha`，默认 0.02，范围 (0, 1]）求解姿态并更新 X 轴基线及陀螺零偏。扰动立即清除静止计时和 EMA 候选；重新满足完整窗口之前保持已锁姿态，不进行陀螺传播。无效样本、断流和离架超时同样清除静止候选，并要求重新确认基线，但不覆写已锁姿态；完整 reset 清除全部标定。
2. 两种模式共用 `launch_ax_threshold_mps2`（默认 14.709975 m/s²）和 `launch_confirm_ms`（默认 20 ms）。加速确认后，`acceleration` 立即触发并锁存；`two_stage` 仅进入待离架状态，冻结校准结果但不启用制导。
3. 仅 `two_stage` 在待离架阶段使用 `launch_release_acceleration_max_mps2`（默认 3 m/s²）及 `launch_release_confirm_ms`（默认 10 ms），低比力连续确认后才触发。比力阈值必须严格低于静止校准区间，单独失重或短冲击不能绕过加速确认。
4. 仅 `two_stage` 使用 `launch_release_timeout_ms`（默认 500 ms）：加速确认后超时未确认离架，取消候选并要求重新静止校准；超时必须大于低比力确认时长，允许恰在边界完成确认。两种模式均在触发前遇到新的无效样本或超过 `attitude_maximum_gap_ms` 的断流时取消候选并要求重校准，不把不连续时长算作有效证据。重复和回退的非负时间戳优先忽略，即使载荷无效也不能清除当前检测证据或打断后续积分。

所有模式都校验完整配置，包括当前模式未使用的三项离架参数，避免切换后出现潜伏的无效配置。

这些默认值依据 2026-10-17 的单次 IMU 记录设定，必须通过更多实测确认；IMU 低比力是离架证据，不是独立硬件离架传感器。记录中的架上加速出现约 58.85 m/s² 的平台，可能存在量程饱和，因此不从该段积分反推离架速度。

### 6.2 默认 PNG 的抛物线 LOS 角速度补偿

`BallisticPngGuidance` 在角速度域扣除自然抛物线基准，不改变角度滤波器的状态。它直接使用滤波器输出的导航系单位方向 `l_N` 及切向导数 `u_N`：

```text
rho = hypot(l_N.x, l_N.z)
q_y_inertial = atan2(l_N.y, rho)
q_dot_y_measured = u_N.y / rho
q_dot_z_effective = (l_N.x*u_N.z - l_N.z*u_N.x) / rho
q_dot_y_parabola = -g0 * rho / (2 * V_actual)
s_y = q_dot_y_measured - q_dot_y_parabola
n_y = N_y * V_actual * s_y / g0
n_z = N_z * V_actual * q_dot_z_effective / g0
```

`V_actual` 使用曝光时刻同步的动态速度。姿态转换和杆臂补偿已在滤波之前完成，不再重复施加机体转动补偿。若测量速率恰好等于自然抛物线速率且横向速率为零，原始弹道请求为零，输出端不叠加保持高度的比力。

`PngCalculationTrace` 记录惯性仰角、测量速率、抛物线基准和补偿后速率；CSV 与 overlay 不扣除该基准。发送端使用最新姿态将原始弹道请求映射到镖体系，再按 `control_channel` 选轴及限幅。速度、视线或最新姿态无效时拒绝输出，不用旧公式回退。

### 6.3 补偿后的惯性系双通道视线角滤波

默认工厂只装配 `PinholeLosEstimator`、`InertialAngleLosFilter` 和 `BallisticPngGuidance`。观测端保留向量接口，先完成安装旋转、曝光姿态转换和可选绝对杆臂补偿，再按原始 `atan2` 公式计算仰角、方位角：

```text
v_C = [1, -(v-cy)/fy, (u-cx)/fx]
l_D = normalize(R_D_from_C * v_C)
l_N_cam = R_N_from_D(t_exposure) * l_D
r_cam_N = R_N_from_D(t_exposure) * [L_x, 0, 0]
R_hat(first) = max(los_estimated_target_distance, 1.0)
R_hat(k) = max(1.0, R_hat(k-1) - (V_exposure(k-1)/2 + V_exposure(k)/2) * dt)
z_N = normalize(l_N_cam + r_cam_N / R_hat(k))  // 杆臂补偿开启
z_N = l_N_cam                                // 杆臂补偿关闭
q_y = atan2(z_N.y, hypot(z_N.x, z_N.z))
q_z = atan2(z_N.z, z_N.x)
```

`PinholeLosEstimator` 检查目标、有限内参、正焦距和 SO(3) 安装旋转。`InertialAngleLosFilter` 防御性单位化输入方向，拒绝非有限、模长小于 1e-12 或溢出的方向。`los_enable_lever_arm_compensation=true` 默认启用绝对杆臂补偿，使用相机相对质心的位置修正视线起点。当前识别结果不含测距，连续段首个有效识别帧使用 `los_estimated_target_distance` 初始化距离；后续按相邻曝光时刻的动态纵向速度梯形积分递减，距离始终不小于 1 m。启用时拒绝负数、非有限速度及积分溢出；关闭时不使用速度和距离，仅跳过杆臂修正，曝光姿态转换及角度滤波保持不变。

俯仰、方位各使用一个独立的二状态 Kalman：`x=[q,dq/dt]`，`F=[1,dt;0,1]`，`H=[1,0]`，`Q=sigma_a_channel²*[dt⁴/4,dt³/2;dt³/2,dt²]`，`R=sigma_q²`。首帧以当前角度、零速率、`P=diag(R,initial_covariance)` 初始化；后续帧直接更新角度观测，不再使用机体角速度前馈。协方差以与原二状态更新等价的 Joseph 形式计算；方位差分、创新和状态正确处理 ±π 跨界。

恢复的 `[guidance]` 参数为 `los_kalman_sigma_a=8`、`los_kalman_sigma_a_pitch=10`、`los_kalman_sigma_a_yaw=3.5`、`los_kalman_sigma_q=0.0035`、`los_initial_covariance=1`、`los_enable_filter=true`。噪声为标准差；通道噪声设为 0 时使用公共值，初始速率方差可为 0。关闭滤波时仍先执行姿态转换和由独立开关控制的杆臂补偿，再计算角度及帧间差分。

下游 `VectorLosEstimate` 不再是六维滤波状态，而是由滤波角度解析生成：`l_N=[cos(q_y)cos(q_z),sin(q_y),cos(q_y)sin(q_z)]`，`u_N` 为该式对时间的导数，天然满足单位长度和切向约束。公共角速率诊断保持 `dq_y/dt`、`cos(q_y)*dq_z/dt` 的定义，不能将未经投影的方位角导数送给 PNG。

首帧不传播或积分占位间隔，只建立角度、距离和速度基准。丢目标、姿态匹配失败、时间异常或 generation 切换后清除连续段；超过 `los_maximum_gap_ms` 时同样重建角度和速度基准，并恢复配置初始距离，不盲目外推制导。`los_camera_lever_arm_x` 仍定义相机相对质心的镖体系 X 轴杆臂；`los_estimated_target_distance` 为每个连续段首帧距离初值，不再固定用于后续帧。方位极点（单位方向水平投影小于 1e-6）无法支持角度状态，滤波器重置并返回无效结果。显示与 CSV 从同一滤波状态生成惯性角度和速率；首帧及滤波不可用时仅显示曝光姿态旋转后的当前观测。六维矢量滤波器、球面投影、各向同性协方差及 `inertial_los_*` 配置均已删除，不保留兼容入口。

### 6.4 镖体系控制通道选择

`[guidance] control_channel=both|pitch|yaw` 在启动时配置发送端通道选择；省略时为 `both`，未知值拒绝加载，不接受带 `_only` 的别名。修改后须重启，不在运行中切换算法状态。

| 值 | 启用输出 | 关闭输出 |
| --- | --- | --- |
| `both` | body-Y/pitch 和 body-Z/yaw | 无，保留既有合成限幅 |
| `pitch` | body-Y/pitch | body-Z/yaw 固定为 0 |
| `yaw` | body-Z/yaw | body-Y/pitch 固定为 0 |

处理顺序为：原制导律及抛物线补偿 → 镖体系指令 → 清零关闭通道 → 过载/加速度限幅。PNG 必须在 roll 转换之后选择镖体系通道，不能直接清零弹道系分量。单通道模式只限制启用轴的绝对指令，关闭轴不消耗额度；上限数值仍沿用所选制导律的现有配置，不增加增益或提高上限。通道选择不是从串口发送前简单裁剪已经合成限幅的双通道结果。

姿态、LOS/滤波和输入有效性检查仍共用完整三维状态；单通道不意味着改成二维几何，也不忽略关闭方向上的非法输入。命令、body 诊断、叠加显示和遥测反映选择后的实际指令；弹道系诊断是同一指令的坐标转换，可能仍有两个非零分量。PNG trace 的 raw 字段保留选择前的原始弹道系结果，`after_overload_limit` 表示选择和过载限幅后、加速度限幅前的弹道系表达，最终 body/navigation 字段与命令一致。串口字节布局、发射/识别标志和失效清零策略不变；仿真复用同一原配置和算法工厂。

## 7. 控制和观测输出接口

制导命令仍由独立的 `CommandSink` 提交；观测输出使用 `RuntimeOutputPorts` 中的窄接口，帧输出消费不可变的 `ProcessedFrame`（包含共享图像所有权、时间戳和 `source_frame_id`）：

- `IVideoRecorder`：`start/reset/record/take_rearm_request/cancel/stop/last_error`。
- `ITelemetryRecorder`：与视频 recorder 相同的生命周期和逐帧记录接口。
- `IVisualizationSink`：`start/reset/publish/cancel/stop/last_error`。
- `IImuRecorder`：接收 `domain::ImuKinematicSample`，以非阻塞 `try_record` 提交，并提供 `SinkDiagnostics`。
- `IAsyncLogger`：接收固定大小的 `RuntimeLogEvent`，提供诊断并在停止时 flush。`AsyncInfoLogger` 只尝试取得生产者锁；锁竞争或队列满时立即拒绝该事件并增加 dropped_count，不等待其他生产者，也不回退到同步 I/O。未注入 logger 或 logger 未运行时不输出日志。启动错误和运行期故障由 `last_error()` 保留，App 决定如何展示。

`SinkDiagnostics` 记录 enabled/healthy、accepted/dropped/failure 计数、队列深度/容量和 sink 自身的 `last_error`。`RuntimeOutputDiagnostics` 分别命名 video、telemetry、visualization、IMU recording 和 logging；未启用 sink 的快照为 disabled、healthy、计数为零。

每个启用的帧 sink 由独立的有界 latest-frame 队列和 worker 服务。队列满时只丢弃该 sink 最旧的待处理帧；`FramePipeline` 仅执行有界队列 fan-out，不在制导/命令路径执行编码、文件或网络 I/O。图像像素通过共享所有权传递，不深拷贝。IMU recorder 和 logger 分别使用独立队列/worker；IMU 队列满仍按既有策略触发运行时故障，日志丢弃、写入或 flush 故障只更新日志诊断，不升级为制导故障。

视频、遥测或可视化 worker 失败仍报告既有 frame-output 故障并取消 runtime；IMU 记录提交失败仍报告运行时故障。该阶段不启用可选输出失败后继续制导的策略。命令 writer 与观测输出队列绝不共用。

串口协议编码适配器将 `GuidanceCommand` 编码为与 `main` 一致的 14-byte 下行帧，不执行 PNG 或坐标变换。body-y/body-z 分别映射 pitch/yaw，按 0.01 g 量化为小端 int16；制导输出无效时 pitch/yaw 编码为零；正常制导的识别位独立取最近提交命令的视觉结果，命令过期时归零。

`SerialCommandSink` 从发送线程启动起按 `[command].send_rate_hz` 周期发送，默认 100 Hz（10 ms），允许 1～1000 Hz；图像采集目标仍为 120 FPS，实际制导计算频率由图像处理决定。

- 生产者使用现有 latest-value mailbox 发布最新结果，发送线程仅采样最新值，不积压 FIFO、不插值。
- 结果更新较慢时重复最近值，但 `[command].command_timeout_ms`（默认 100 ms，必须大于 0）从新命令提交的单调时刻计算，既不从首次消费时刻计算，也不因重发而续期；来源图像时间和 frame ID 不变。该超时检测命令生产断流，不替代曝光时间、IMU 匹配和输入有效性检查。
- 正常制导超时后清零过载及识别位，保留最近命令的发射标志；未发射时始终发送零过载。新代际及重启清除缓存，正常制导收到新命令前发送零过载及零状态标志。
- 取消或停止会唤醒发送线程并停止周期发送；串口写失败计数保留，调度或写入延迟只跳过错过的周期，不突发补发。

100 Hz 是调度目标而非硬实时保证，断线、进程退出后的执行端行为仍须由下位机 watchdog 保证。`log` 输出及仿真命令 sink 仍按新结果处理，不做周期重发。

输出映射器的会话状态必须显式重置：`ICommandOutputMapper::reset()` 清除调试目标锁存、首次触发时间和缓存代际，保留配置及物理限制；首次触发时间的零值合法。`CommandOutputProcessor::reset()` 同时清空映射器状态、输出快照和尝试编号，允许新会话重用相同 generation；`invalidate()` 清空映射器状态和快照，但保持尝试编号递增。串口、日志、仿真输出的启动均走该重置路径，换代在各输出端的发送屏障内清理；重置不得与在途 `prepare()/complete()` 并发。不能仅依赖 generation 数字变化隔离两次运行。

固定过载模式 `launch_delay` 和 `target_detected` 的发射位均来自当前代际的最新 IMU 状态，不依赖视觉邮箱存在命令。触发前输出发射位随 IMU、识别位 `0` 和零过载，保留下位机原上升流程；`launch_delay` 仅从 IMU 发射确认时间计时，程序启动后的等待不消耗延迟，提前识别不置识别位也不改变计时。`target_detected` 仅由未过期且自身标记为发射后的识别结果首次触发，不要求该帧 LOS 速率或制导请求有效。触发后识别位为 `1` 并发送限幅后的固定过载；识别模式的目标丢失或邮箱超时不撤销已触发锁存。持续时间结束后过载归零，识别模式保留识别锁存位，延迟模式识别位归零；会话重置清除识别锁存。两种模式仍受代际、IMU 发射状态和停止／故障门控约束，门控拒绝时不输出非零过载或调试识别位。

| 字节位置（从 0 开始） | 含义 |
| --- | --- |
| 0..1 | 帧头 `AA 55` |
| 2 | 发射标志：所选发射判据未确认为 `0`，确认后为 `1`（原 Roll 字节） |
| 3 | 正常制导识别位：有效期内结果识别到目标为 `1`，未识别、未提交或超时为 `0`；固定模式按上述触发契约 |
| 4..5 | Yaw 过载 |
| 6..7 | Pitch 过载 |
| 8..9 | 保留，固定 `0` |
| 10..11 | CRC-16/CCITT-FALSE，小端，覆盖 bytes 2..9（含发射标志和识别位） |
| 12..13 | 帧尾 `0D 0A` |

正常制导的状态位行为如下。发射标志采用 `launch_detection_mode` 所选判据：`acceleration` 加速确认即置 `1`，`two_stage` 还须离架确认；触发后本周期保持为 `1`，复位后为 `0`。目标丢失、制导失效或故障清零均不清除已触发状态。触发后的首个有效图像仅建立速率基准，即使识别成功也发送识别位 `1` 和零过载；后续目标丢失时识别位恢复为 `0`。触发前不提交制导命令，但串口发送线程仍周期发送发射标志 `0` 和零过载的控制帧；曝光时刻早于确认的历史图像，包括插值区间跨触发边界的图像，不得借用最新 IMU 的已触发状态更新命令。周期帧的发射标志来自最近提交的当前代际命令，不因没有新图像而自行推断 IMU 状态。运行时故障清零时识别位为 `0`。下位机须按此布局解析，不再将 bytes 2..3 解释为双字节 Roll 过载。20-byte IMU 上行帧及字段转换不变。

## 8. 运行时组装和输出模式

App 是唯一 Composition Root。`make_runtime_output_ports` 按既有 `output.mode` 装配：

| `output.mode` | 帧输出 | 其他输出 |
|---|---|---|
| `debug` | 连续本地视频 + 逐帧遥测 CSV | IMU CSV + 异步日志 |
| `match` | 起飞窗口本地视频 + 逐帧遥测 CSV | IMU CSV + 异步日志 |
| `rtsp` | RTSP 可视化 | IMU CSV + 异步日志 |

`RuntimeSupervisor` 持有输出端口并统一管理 start/cancel/close/join/stop 生命周期。`FramePipeline` 向每个帧输出 worker 独立提交同一 processed-frame snapshot；`ImuPipeline` 将 IMU 样本提交给独立 recorder。match 模式的视频与遥测 sink 共享路径及 generation/window 协调元数据，但保留独立队列、writer、错误状态；只有两者均成功完成窗口后才发出 rearm。各文件可能因独立队列丢帧而包含不同帧子集，使用遥测 CSV 的 `source_frame_id` 关联原始帧，不依赖行号或视频帧序号一致。

逐帧 CSV 与录像／RTSP 的 LOS 展示共用导航惯性系快照：`los_inertial_q_y_rad` 为仰角，`los_inertial_q_z_rad` 为方位角；有效滤波帧从同一份滤波方向与导数产生角度、角速度，包含杆臂修正，但不扣除制导律局部使用的抛物线基准。`los_inertial_rate_q_y_rad_s` 为仰角导数，`los_inertial_rate_q_z_rad_s` 保持横向有效角速率 `cos(q_y) * dq_z/dt` 的定义，不是直接的方位角导数。触发前、建基准帧及滤波速率不可用时，角度仅为用曝光姿态旋转到导航系的当前观测，速率有效位为 0；没有观测或处于仰角极点时不得将占位零值解释为有效角度。画面标注 `Inertial LOS` 和 `Inertial LOS rate`，单位分别为 ° 和 °/s。

逐帧 CSV 用 `imu_roll_rad,imu_pitch_rad,imu_yaw_rad` 替换原三轴 IMU 角速度列，同时移除重复的旧 pitch/roll 列。三轴角与画面的 `Exposure attitude` 都从插值到 `frame_timestamp_ns` 的同一完整姿态矩阵提取，不使用发送时刻的最新姿态，也不直接积分日志中的角速度。欧拉角约定为 `R_N_from_D = R_Y(-yaw) R_Z(pitch) R_X(roll)`；俯仰极点选取 yaw=0 的等价姿态。CSV 单位 rad，画面单位 °；姿态无效时 CSV 三列留空且 `imu_attitude_valid=0`，画面显示 N/A。yaw 是局部导航系相对航向，不是绝对航向。本次 CSV 字段结构不保留旧别名，读取方须按表头取列；全量原始 IMU CSV 的三轴角速度及其字段结构保持不变。

录像与 RTSP 在左上角共用 `Lever-arm distance: 12.34 m` 距离展示，位于两行 LOS 数据之后，保留两位小数。距离直接来自滤波器 `sample()` 的本帧实际补偿距离，经 `GuidanceOverlay` 和显示映射传递，不在可视化端重新积分，也不依赖角速率有效位：首帧建基准后即可显示配置初值，后续显示积分后的距离，下限为 `1.00 m`；关闭杆臂补偿、尚未初始化、丢失目标或输入异常导致状态重置时显示 `N/A`，不沿用上一帧距离。读取诊断快照不推进时间、不改变补偿或滤波行为。

录像与 RTSP 使用同一绘图实现：保留搜索 ROI 的红色边框、目标蓝色边框和中央姿态十字；左上角数据及底部目标状态文字统一为白色，不再绘制参考圆或目标中心红色标记。目标中心坐标仍用于 LOS 计算和 CSV 记录；识别面积阈值仍用于检测，但不再通过结果结构传递参考圆的绘图参数。

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
- **观测坐标与时间**：`runtime_supervisor_test` 验证建基准帧的导航系角度及有效滤波帧的同源 LOS 角度/速率；`output_mapping_test` 验证曝光矩阵的三轴姿态提取、时间插值和俯仰极点约定；`local_recorders_test` 验证新 CSV 表头、无效姿态留空及 CSV 与显示数据一致。
- **记录器和故障语义**：`local_recorders_test` 检查视频/CSV、窗口、部分停止及 rearm；`output_sink_diagnostics_test` 和 `imu_pipeline_test` 检查 IMU 队列故障传播以及日志丢弃/写入/flush 故障不升级为 runtime fault。
- **模式组合**：`mode_composition_test` 验证 debug/match/rtsp 组合、所有模式的 IMU recorder/logger、路径分配，以及 RTSP 适配器的懒启动/错误/重置；无线关闭时验证拒绝真实 RTSP 组装。
- **发射触发**：`launch_detection_test`、`attitude_adapter_test` 验证默认加速触发、可选两阶段确认、误触发拒绝、超时边界、断流/异常样本及复位重校准；`body_vector_runtime_test` 在两种模式及两种 PN 下验证触发前不建立飞行滤波状态、跨触发边界不下发、使用触发后的动态速度；默认算法和仿真测试覆盖参数解析、缺省/非法模式、确认时刻元数据及两种模式下真实 Socket 录制链路。
- **控制通道**：`guidance_adapter_test`、`body_vector_pn_guidance_test` 验证滚转后的单通道选择、关闭轴精确置零、正负指令、两级限幅及诊断一致性；`default_guidance_algorithms_test` 验证 `both/pitch/yaw` 解析、缺省、非法值和两种算法的工厂输出；`png_calculation_trace_test` 验证各通道下只读追踪不改变计算结果。
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
- `SimulationRunner::timing()` / `output_diagnostics()` 只读封装原诊断。`timing()` 通过监督器所有权互斥锁与创建、销毁同步，可由其他线程查询，但运行器对象须保持存活；它不是无等待接口。其他公开方法、输出诊断和析构均由同一生命周期线程调用。finish 先 stop/join、编码/刷新，再报告各 sink 的 accepted/dropped/failures/last_error，最后清水位并销毁 session。配置 Hz 与主机实测吞吐分别报告；SimulationCameraSource 不把握手标称 Hz 冒充实际 capture FPS。
- 原 debug/match 输出使用独立运行目录，保存 AVI、遥测 CSV 和全程 IMU CSV。match 由原起飞判据自然触发，未满窗口正常停止也保存部分数据；下一 session 不覆盖上一发。当前 640×480 / 120 Hz、match 4000 ms 的 BGR 窗口约 442 MB，内存上限仍为 1,105,920,000 bytes，超限启动前拒绝；默认父目录 `/home/panshenghe/dart_sim_ws/recordings/`。

`simulation_config_test`、`simulation_contract_test` 验证原配置与握手；`simulation_sources_test`、`simulation_command_sink_test`、`simulation_sync_test`、`simulation_runner_test` 覆盖适配器、丢流/溢出、同步屏障、原插值拒绝及生命周期。`simulation_recording_test` 使用真实 Socket 来源、原算法和原输出，验证两发 match 部分落盘、完整 IMU、debug/prelaunch、文件隔离及错误可见，临时文件由 RAII 清理。这些不构成 Gazebo 闭环验收。

实际步骤、DSIM v1 和验收状态见 SIM 仓库 `docs/mosas-simulation.md`。2026-10-14 的实际尝试受无可用 X/Ogre 渲染及 DDS 服务未 ready 阻塞，没有发射/实际录制；历史 PNG 非零执行证据采用旧参数，不能替代当前配置验收。

## 11. 原程序负载下的相机／IMU 标定采集

实际场上运行由 `main/mosas_guidance_app.cpp` / `scripts/run_guidance.sh` 启动，默认读取 `config/mosas_guidance.conf`；标定采集由独立的 `main/mosas_sync_calibration_app.cpp` / `scripts/run_sync_calibration.sh` 启动，默认读取 `config/mosas_sync_calibration.conf`。两个入口均为无参数 `main()`，不再解析命令行参数。两个配置文件各自包含完整的设备、算法和输出设置，互不引用；标定文件另有 `[sync_calibration] output_dir/seconds/fps/max_mib`，两者各自通过 `[app] enable_timing_statistics` 控制退出统计。标定目录的相对路径按配置文件所在目录解析，只在标定入口执行采集预检。源码脚本仅保留构建选项，切到项目根目录后无参数启动程序；直接启动源码构建的二进制也须在项目根目录执行。安装后脚本直接启动同目录的可执行文件，不调用 CMake；程序通过 Linux `/proc/self/exe` 定位 CMake 安装配置目录，默认是 `<prefix>/etc/mosas/`，不依赖启动工作目录。安装配置目录或文件缺失时明确失败，不回退到当前目录的配置。公共 `device_application` 支持负责设备初始化、原算法／输出组装和统计，各入口独立选择命令 sink 并控制生命周期。

标定入口通过输入源装饰器旁路记录，不增加第二个相机／串口读取者，不增加人工压力，不改变原算法及源时间补偿。它只构造无设备资源的内存 CommandSink，不调用生产命令工厂，连清零帧也不发送；执行器仍须物理断能。场上入口按原配置构造生产命令 sink，不含标定分支。

原 SDK 图像在 prepare 前独立拷贝，后台转灰度写入 version=2 数据目录；不能共享会被 prepare 原地翻转的像素。相机倒装 180°且现有内参来自正立图像，离线分析复现原 resize／180° flip 后使用该内参。IMU 全速记录未经估计器修正的数据，图像默认抽样 30 Hz，均保留整数纳秒的运行时间和逆补偿时间。

旁路队列／文件有界，记录溢出、数据错误、预算超限、文件关闭失败及中断均不能写 complete=true。原生产者退出、记录队列排空并成功关闭文件后才原子完成元数据。报告保留接收诊断、原阶段实际样本数和记录开销；停发 TX、缺失发射后分支及额外记录负载限制必须明确，不宣称完全等价于所有实际运行状态或硬件同步。

`device_application_entry_test` 验证源码入口的默认文件隔离、预检和失败清理；`device_application_install_test` 将真实安装产物放入临时前缀，从其他工作目录验证无参数启动、采集目录解析，以及缺失安装配置时不回退。测试只使用不存在的串口，临时产物在结束后清理。

使用步骤、内参方向、文件格式、偏移符号及拒绝条件见 [`CAMERA_IMU_SYNC_CALIBRATION.md`](CAMERA_IMU_SYNC_CALIBRATION.md)。
