#ifndef MOSAS_RUNTIME_ASYNC_INFO_LOGGER_HPP
#define MOSAS_RUNTIME_ASYNC_INFO_LOGGER_HPP

#include <mosas/runtime/log_event.hpp>
#include <mosas/runtime/output_ports.hpp>
#include <mosas/runtime/runtime_types.hpp>
#include <mosas/runtime/spsc_ring.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <mutex>
#include <string>
#include <thread>

namespace mosas::runtime {

class AsyncInfoLogger final : public IAsyncLogger {
public:
    /**
     * @brief 创建异步日志器。
     * @param output 后台 worker 写入的流；流对象须比 logger 活得更久。
     * @param queue_capacity 有界日志事件队列容量；满队列时新日志丢弃并计数。
     */
    explicit AsyncInfoLogger(std::ostream& output,
                             std::size_t queue_capacity = 256);
    ~AsyncInfoLogger();

    AsyncInfoLogger(const AsyncInfoLogger&) = delete;
    AsyncInfoLogger& operator=(const AsyncInfoLogger&) = delete;

    /**
     * @brief 启动日志后台 worker。
     * @return 成功创建 worker 时为 true。
     */
    bool start() override;

    /**
     * @brief 非阻塞地提交结构化日志事件。
     * @param event 固定大小的日志事件；包含单调时间戳和消息内容。
     * @return 事件被接收时为 true；生产者锁竞争、队列满或 logger 已停止时为 false。
     * @note 不等待其他生产者；锁竞争和队列满均丢弃本次事件并增加 dropped_count。
     */
    bool try_submit(const RuntimeLogEvent& event) noexcept override;

    /**
     * @brief 将文本消息封装为日志事件并非阻塞提交。
     * @param timestamp_ns 事件发生时的单调时钟纳秒值。
     * @param message 文本消息；过长内容可能按固定容量截断。
     * @return 事件被队列接收时为 true。
     */
    bool try_submit_text(TimestampNs timestamp_ns,
                         const std::string& message) noexcept;

    /** @brief 获取队列、丢弃计数和 worker 健康状态。 @return 日志 sink 诊断快照。 */
    SinkDiagnostics diagnostics() const noexcept override;

    bool running() const noexcept {
        return running_.load(std::memory_order_acquire);
    }

    /**
     * @brief 停止后台 worker，并刷新已接收的日志事件后返回。
     * @note 可阻塞；只应在非实时生命周期管理线程调用。
     */
    void stop_and_flush() noexcept override;

private:
    void worker_loop() noexcept;
    void set_error(const char* message) noexcept;

    std::ostream* output_ = nullptr;
    // 多个 worker 只尝试取得生产者锁，保持底层环形队列单生产者语义且不阻塞控制链。
    std::mutex producer_mutex_;
    SpscRing<RuntimeLogEvent> queue_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> accepted_count_{0};
    std::atomic<std::uint64_t> dropped_count_{0};
    std::atomic<std::uint64_t> failure_count_{0};
    std::atomic<bool> healthy_{true};
    mutable std::mutex error_mutex_;
    std::string last_error_;
    std::thread worker_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_ASYNC_INFO_LOGGER_HPP
