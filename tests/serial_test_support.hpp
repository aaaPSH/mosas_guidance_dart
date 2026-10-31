#ifndef MOSAS_TEST_SERIAL_SUPPORT_HPP
#define MOSAS_TEST_SERIAL_SUPPORT_HPP
#include <mosas/adapters/host_adapters.hpp>
#include <serial_package/serial_protocol.hpp>
#include "command_output_test_support.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <vector>
#include <thread>
namespace mosas::test {
using SerialClock=std::chrono::steady_clock;
using ControlFrame=std::array<std::uint8_t,serial_package::kControlFrameSize>;
/** @brief 真实串口写路径的无硬件 PTY 夹具；保留超时读的部分帧。 */
class SerialFixture {
public:
    ~SerialFixture(){port->close();if(master_>=0)::close(master_);}
    bool open(){
        master_=::posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
        if(master_<0||::grantpt(master_)||::unlockpt(master_))return false;
        const auto slave=::ptsname(master_);if(!slave)return false;
        serial_package::SerialPortConfig config;config.device=slave;return port->open(config);
    }
    /** @return 在限定时间内取得一帧时为 true，超时不丢弃已读字节。 */
    bool read_frame(ControlFrame& frame,SerialClock::duration timeout=std::chrono::milliseconds(500)) {
        const auto deadline=SerialClock::now()+timeout;
        while(SerialClock::now()<deadline){
            pollfd descriptor{master_,POLLIN,0};
            const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-SerialClock::now()).count();
            const int ready=::poll(&descriptor,1,std::max(1,static_cast<int>(remaining)));
            if(ready<0&&errno==EINTR)continue;
            if(ready<=0||(descriptor.revents&POLLIN)==0)return false;
            const auto size=::read(master_,partial_.data()+filled_,partial_.size()-filled_);
            if(size<0&&(errno==EINTR||errno==EAGAIN))continue;
            if(size<=0)return false;
            filled_+=static_cast<std::size_t>(size);
            if(filled_==partial_.size()){frame=partial_;filled_=0;return true;}
        }
        return false;
    }
    bool wait_for(const ControlFrame& expected,SerialClock::duration timeout=std::chrono::milliseconds(500)) {
        const auto deadline=SerialClock::now()+timeout;ControlFrame frame{};
        while(read_frame(frame,deadline-SerialClock::now()))if(frame==expected)return true;
        return false;
    }
    std::vector<ControlFrame> drain(){
        std::vector<ControlFrame> result;ControlFrame frame{};
        while(read_frame(frame,std::chrono::milliseconds(2)))result.push_back(frame);
        return result;
    }
    std::shared_ptr<serial_package::SerialPort> port=std::make_shared<serial_package::SerialPort>();
private:
    int master_=-1;ControlFrame partial_{};std::size_t filled_=0;
};
/** @return 带 CRC 的协议参考帧；yaw=body-Z，pitch=body-Y，单位 g。 */
inline ControlFrame encoded(bool launched,bool target,double yaw=0,double pitch=0){
    ControlFrame frame{};serial_package::encode_control_frame({launched,target,yaw,pitch},&frame);return frame;
}
/** @return 与真实输出匹配的诊断快照；串口读到字节不代表发送线程已发布诊断。 */
template<class Predicate>
std::optional<domain::CommandEmissionSnapshot> wait_for_emission(runtime::CommandSink& sink, Predicate predicate) {
    const auto deadline=SerialClock::now()+std::chrono::milliseconds(500);
    while(SerialClock::now()<deadline) {
        auto value=sink.latest_emission();
        if(value&&predicate(*value))return value;
        std::this_thread::yield();
    }
    return {};
}
/** @brief 输出测试的原子可控状态源；不是生产预测或时间补偿实现。 */
struct LiveOutputState {
    std::atomic<std::uint64_t> generation{0}, reads{0};
    std::atomic<double> roll{0};
    std::atomic<bool> available{true},valid{true};
    std::atomic<std::int64_t> offset_ns{-1'000'000};
    std::optional<domain::FlightStateSnapshot> snapshot(){
        ++reads;if(!available)return {};
        auto state=command_test_state(runtime::monotonic_timestamp_ns()+offset_ns.load(),generation.load(),roll.load());
        state.attitude.valid=valid.load();return state;
    }
};
/** @return 带真实映射、同轴当前时钟的输出绑定。 */
inline bool bind_live_output(runtime::CommandSink& sink,LiveOutputState& state,guidance::CommandMappingConfig config={}) {
    return bind_test_output(sink,[&state]{return state.snapshot();},[]{return runtime::monotonic_timestamp_ns();},config);
}
/** @return 元数据完整的当前代际弹道测试请求。 */
inline domain::GuidanceCommand fresh_command(double y=0.23,double z=-0.11,std::uint64_t generation=0){
    domain::GuidanceCommand command;command.source_frame_id=1;command.generation=generation;
    command.source_time=command_test_time(runtime::monotonic_timestamp_ns());
    command.valid=command.launched=command.target_found=true;command.invalid_reason=domain::GuidanceInvalidReason::none;
    command.overload_g={0,y,z};return command;
}
}
#endif
