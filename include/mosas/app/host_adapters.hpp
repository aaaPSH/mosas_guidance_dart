#ifndef MOSAS_APP_HOST_ADAPTERS_HPP
#define MOSAS_APP_HOST_ADAPTERS_HPP

#include <mosas/app/app_config.hpp>
#include <serial_package/serial_port.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <string>

namespace mosas::app {

class SimulatedImuSource final : public runtime::ImuSource {
public:
    explicit SimulatedImuSource(double sample_rate_hz);

    runtime::SourceResult read(runtime::ImuSample* sample) override;
    void cancel() noexcept override;

private:
    std::chrono::nanoseconds period_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::chrono::steady_clock::time_point next_deadline_{};
    bool started_ = false;
    bool cancelled_ = false;
};

class LoggingCommandSink final : public runtime::CommandSink {
public:
    LoggingCommandSink();
    explicit LoggingCommandSink(std::ostream& output);

    bool send(const runtime::GuidanceCommand& command) override;

private:
    std::ostream* output_;
    std::mutex mutex_;
};

class SerialCommandSink final : public runtime::CommandSink {
public:
    explicit SerialCommandSink(
        std::shared_ptr<serial_package::SerialPort> port);
    SerialCommandSink(std::shared_ptr<serial_package::SerialPort> port,
                      std::ostream& output);

    bool send(const runtime::GuidanceCommand& command) override;

    std::string last_error() const;

private:
    bool fail(const std::string& message);

    std::shared_ptr<serial_package::SerialPort> port_;
    std::ostream* output_ = nullptr;
    mutable std::mutex mutex_;
    std::string last_error_;
};

}  // mosas::app 命名空间结束

#endif  // MOSAS_APP_HOST_ADAPTERS_HPP
