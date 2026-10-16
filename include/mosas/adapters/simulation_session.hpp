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
/** @brief Convert camera installation axes by A*R*A^T; rejects non-rotations. */
domain::Rotation3<domain::DartFrameTag,domain::CameraFrameTag>
simulation_camera_rotation(const std::array<double,9>& rotation);
/** @brief Single-use connection session shared by all four simulation ports.
 * One pump drains transport independently of algorithm workers. Failure clears queues,
 * wakes consumers and closes both channels; reconnect requires a fresh object/runtime.
 */
class SimulationSession {
public:
    using Watermark=std::function<std::optional<runtime::ImuStateSnapshot>()>;
    struct Counters {std::uint64_t images=0,imus=0,applied=0,nonzero_applied=0;};
    /** @brief Received sensor and physically acknowledged execution counts. */
    Counters counters()const;
    explicit SimulationSession(const std::string& directory);
    ~SimulationSession();
    SimulationSession(const SimulationSession&)=delete;
    SimulationSession& operator=(const SimulationSession&)=delete;
    bool wait_ready(std::chrono::milliseconds timeout);
    bool ready() const;
    bool failed() const;
    bool cancelled() const;
    std::string error() const;
    dart_sim_transport::ServerSnapshot metadata() const;
    std::int64_t now_ns() const noexcept;
    /** @brief Nonblocking pop and independently cancelable IMU wait. */
    bool pop_imu(runtime::ImuAngularVelocitySample* sample) noexcept;
    bool wait_imu() noexcept;
    runtime::ImuSourceDiagnostics imu_diagnostics() const noexcept;
    /** @brief Take newest owning RGB message; older waiting images are replaced, not queued. */
    std::optional<dart_sim_transport::Message> take_image();
    /** @brief Short wait for clock/data/cancellation; never waits for an IMU consumer. */
    void wait_change(std::chrono::milliseconds timeout);
    /** @brief Bind only this runtime's processed watermark; clear after stop/join.
     * Invocation and rebinding are serialized, so no callback can outlive clear_watermark().
     */
    void bind_watermark(Watermark callback);
    void clear_watermark();
    std::optional<runtime::ImuStateSnapshot> processed_state();
    bool submit(dart_sim_transport::Overload overload,std::int64_t source_ns);
    void discard_commands();
    /** @brief Nonblocking cancellation. stop() performs the eventual joins. */
    void cancel() noexcept;
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
