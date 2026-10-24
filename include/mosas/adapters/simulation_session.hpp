#pragma once
#include <dart_sim_transport/endpoint.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

namespace mosas::adapters {
/**
 * @brief 通过 A*R*A^T 将相机安装旋转从仿真轴约定转换为镖体轴约定。
 * @param rotation 按行排列的 3×3 仿真安装旋转矩阵。
 * @return 从相机系到镖体系的旋转。
 * @throws std::invalid_argument 输入非有限、非正交或包含反射时抛出。
 */
domain::Rotation3<domain::DartFrameTag,domain::CameraFrameTag>
simulation_camera_rotation(const std::array<double,9>& rotation);
/**
 * @brief 由四个仿真端口共享的一次性连接会话，独立接收线程持续消费传输数据。
 * @note 故障会清空等待数据并唤醒消费者；重新连接必须新建会话及运行时。
 *       数据与状态查询通过内部互斥同步；IMU 样本仅允许一个消费者。
 *       stop() 与析构须在生命周期线程串行调用，不得并发 join 同一接收线程。
 */
class SimulationSession {
public:
    using Watermark=std::function<std::optional<runtime::ImuStateSnapshot>()>;
    struct Counters {std::uint64_t images=0,imus=0,applied=0,nonzero_applied=0;};
    /** @brief 查询传感器接收及下位端实际确认执行的计数。 @return 线程安全的计数快照。 */
    Counters counters()const;
    /**
     * @brief 创建传输端点并立即启动后台接收线程；不等待连接就绪。
     * @param directory 仿真传输使用的 Unix Socket 目录。
     * @note 对象须保持存活，直到所有使用它的端口及水位回调停止工作。
     */
    explicit SimulationSession(const std::string& directory);
    /** @brief 停止并回收传输线程，再清除水位回调；可能阻塞。 */
    ~SimulationSession();
    SimulationSession(const SimulationSession&)=delete;
    SimulationSession& operator=(const SimulationSession&)=delete;
    /**
     * @brief 等待握手成功、故障或取消，最多等待指定的主机时钟时长。
     * @param timeout 最大等待时长，不是仿真时间预算。
     * @return 连接已就绪且未故障、未取消时为 true；超时或会话结束时为 false。
     */
    bool wait_ready(std::chrono::milliseconds timeout);
    bool ready() const;
    bool failed() const;
    bool cancelled() const;
    std::string error() const;
    /** @brief 查询握手及会话元数据。 @return 加锁复制的传输状态快照。 */
    dart_sim_transport::ServerSnapshot metadata() const;
    /** @brief 查询最新会话仿真时间。 @return 纳秒值；握手前为 0，不代表主机单调时间。 */
    std::int64_t now_ns() const noexcept;
    /**
     * @brief 尝试取出一个 IMU 样本，不等待新样本到达。
     * @param sample 非空输出指针。
     * @return 已取出样本时为 true；无数据、故障、取消或空指针时为 false。
     * @note 由唯一 IMU 消费者调用；读取队列时会取得内部互斥锁。
     */
    bool pop_imu(runtime::ImuAngularVelocitySample* sample) noexcept;
    /** @brief 等待 IMU 样本或会话结束。 @return 仍可消费时为 true，故障或取消时为 false。 */
    bool wait_imu() noexcept;
    runtime::ImuSourceDiagnostics imu_diagnostics() const noexcept;
    /** @brief 取走最新拥有型 RGB 消息，较旧等待帧会被替换。 @return 无可用图像时为空。 */
    std::optional<dart_sim_transport::Message> take_image();
    /** @brief 等待时钟、数据或取消变化，不等待 IMU 消费者。 @param timeout 主机时钟等待上限。 */
    void wait_change(std::chrono::milliseconds timeout);
    /**
     * @brief 绑定当前运行时已处理状态的查询回调；运行时 stop/join 后须清除。
     * @param callback 返回当前运行时处理水位的函数；空函数表示解除绑定。
     * @note 调用回调与重新绑定串行化；clear_watermark() 返回后不会留下在途回调。
     *       回调在水位互斥锁内执行，不得递归绑定、清除或查询同一水位。
     */
    void bind_watermark(Watermark callback);
    /** @brief 清除水位回调并等待在途调用结束，调用后才能销毁回调捕获的运行时。 */
    void clear_watermark();
    /**
     * @brief 在水位互斥锁内调用当前运行时的状态查询回调。
     * @return 已处理 IMU 状态；未绑定回调或运行时尚无状态时为空。
     * @note 可能等待在途查询或绑定操作，不得从同一水位回调中递归调用。
     */
    std::optional<runtime::ImuStateSnapshot> processed_state();
    /**
     * @brief 将一条仿真控制指令交给传输端，不将主机时间写入仿真协议。
     * @param overload 仿真左向、上向过载，单位 g，并携带有效性、原因码及来源帧信息。
     * @param source_ns 原始图像的会话仿真时间，单位 ns。
     * @return 会话就绪且传输端接受时为 true；未就绪或提交失败时为 false。
     */
    bool submit(dart_sim_transport::Overload overload,std::int64_t source_ns);
    /** @brief 丢弃传输端尚未发送的控制命令，用于会话或起飞代际切换。 */
    void discard_commands();
    /** @brief 请求取消、清空等待数据并唤醒读者；不等待传输线程退出。 */
    void cancel() noexcept;
    /** @brief 请求取消并 join 接收线程；仅在生命周期线程调用，可能阻塞。 */
    void stop() noexcept;
private:
    void pump() noexcept;
    void accept(dart_sim_transport::Message message);
    void fail(std::string error) noexcept;
    dart_sim_transport::Endpoint endpoint_;
    mutable std::mutex mutex_;std::condition_variable cv_;
    bool ready_=false,cancelled_=false,failed_=false;
    std::string error_;
    dart_sim_transport::ServerSnapshot metadata_;
    std::int64_t now_=0,last_imu_=-1,last_clock_=-1,last_image_=-1,image_enabled_since_=-1;
    std::deque<runtime::ImuAngularVelocitySample> imu_;
    std::optional<dart_sim_transport::Message> image_;
    std::uint64_t delivered_=0;
    Counters counters_;
    bool valid_imu_seen_=false;
    std::mutex watermark_mutex_;Watermark watermark_;
    std::thread worker_;
};
} // namespace mosas::adapters
