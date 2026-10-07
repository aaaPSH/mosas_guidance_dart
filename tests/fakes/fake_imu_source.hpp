#ifndef MOSAS_TEST_FAKES_FAKE_IMU_SOURCE_HPP
#define MOSAS_TEST_FAKES_FAKE_IMU_SOURCE_HPP

#include <mosas/runtime/runtime_interfaces.hpp>

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <vector>

namespace mosas::test {

class FakeImuSource final : public runtime::ImuSource {
public:
    explicit FakeImuSource(
        std::vector<runtime::ImuAngularVelocitySample> samples);

    runtime::SourceResult start() override;
    bool try_pop(runtime::ImuAngularVelocitySample* sample) noexcept override;
    bool wait_for_data() noexcept override;
    void cancel() noexcept override;
    void stop() noexcept override;

    void release() noexcept;

private:
    std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<runtime::ImuAngularVelocitySample> samples_;
    std::size_t next_sample_ = 0;
    bool started_ = false;
    bool released_ = false;
    bool cancelled_ = false;
};

}  // namespace mosas::test

#endif  // MOSAS_TEST_FAKES_FAKE_IMU_SOURCE_HPP
