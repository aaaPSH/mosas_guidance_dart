#include "serial_test_support.hpp"
#include "test_support.hpp"
#include <condition_variable>
#include <future>
#include <mutex>
#include <sstream>
namespace {
using namespace mosas;
using namespace mosas::test;
using namespace std::chrono_literals;
/** @brief 只提交一次请求，验证在线重发随最新 Roll 改变，不缓存旧 body 结果。 */
int dynamic_roll(){
    SerialFixture serial;CHECK(serial.open());LiveOutputState state;std::ostringstream errors;
    app::CommandAppConfig config;config.command_timeout_ms=1000;
    app::SerialCommandSink sink(serial.port,config,errors);CHECK(bind_live_output(sink,state));CHECK(sink.start());
    CHECK(serial.wait_for(encoded(false,false)));
    const auto request=fresh_command(0.5,0);CHECK(sink.try_submit(request));
    CHECK(serial.wait_for(encoded(true,true,0,0.5)));
    const auto first=wait_for_emission(sink,[](const auto& value){return value.mapping.command.valid&&value.mapping.command.source_frame_id==1;});
    CHECK(first&&first->mapping.command.valid);
    state.roll=std::acos(-1.0)/2;
    CHECK(serial.wait_for(encoded(true,true,-0.5,0),200ms));
    auto second=wait_for_emission(sink,[](const auto& value){return value.mapping.command.valid&&std::abs(value.mapping.command.overload_g.z+0.5)<1e-12;});
    CHECK(second&&second->attempt_id>first->attempt_id);
    CHECK(second->mapping.command.source_frame_id==first->mapping.command.source_frame_id);
    CHECK(second->mapping.command.source_time.monotonic_ns==first->mapping.command.source_time.monotonic_ns);
    CHECK(second->mapping.trace.attitude.time.monotonic_ns>first->mapping.trace.attitude.time.monotonic_ns);
    CHECK(sink.diagnostics().submitted_count==1);
    state.available=false;
    CHECK(serial.wait_for(encoded(true,true),200ms));
    state.available=true;state.offset_ns=1'000'000'000;
    serial.drain();CHECK(serial.wait_for(encoded(true,true),100ms));
    state.offset_ns=-100'000'000;
    serial.drain();CHECK(serial.wait_for(encoded(true,true),100ms));
    state.offset_ns=-1'000'000;
    CHECK(serial.wait_for(encoded(true,true,-0.5,0),200ms));
    // 控制失效是新请求，必须覆盖旧非零请求而不是等其原 timeout。
    auto invalid=request;invalid.valid=false;invalid.invalid_reason=domain::GuidanceInvalidReason::invalid_rate;
    invalid.overload_g={};invalid.source_frame_id=2;
    CHECK(sink.try_submit(invalid));CHECK(serial.wait_for(encoded(true,true),200ms));
    const auto zero=wait_for_emission(sink,[](const auto& value){return value.mapping.command.source_frame_id==2;});
    CHECK(zero&&zero->mapping.command.source_frame_id==2);
    CHECK(!zero->mapping.command.valid&&zero->mapping.command.target_found);
    for(int i=0;i<2;++i){ControlFrame frame;CHECK(serial.read_frame(frame));CHECK(frame==encoded(true,true));}
    sink.stop();CHECK(errors.str().empty());return 0;
}
/** @brief 重发与状态刷新均不得延长原始提交的有效期。 */
int unchanged_lease(){
    SerialFixture serial;CHECK(serial.open());LiveOutputState state;std::ostringstream errors;
    app::CommandAppConfig config;config.command_timeout_ms=80;
    app::SerialCommandSink sink(serial.port,config,errors);CHECK(bind_live_output(sink,state));CHECK(sink.start());
    const auto submitted=SerialClock::now();CHECK(sink.try_submit(fresh_command(0.4,0)));
    CHECK(serial.wait_for(encoded(true,true,0,0.4)));
    state.roll=std::acos(-1.0)/2;
    CHECK(serial.wait_for(encoded(true,true,-0.4,0)));
    CHECK(serial.wait_for(encoded(true,false),200ms));
    CHECK(SerialClock::now()-submitted>=60ms);
    CHECK(sink.diagnostics().submitted_count==1);
    sink.stop();return 0;
}
/** @brief 状态读取与切代交错时，复位屏障等待在途写入且不反向获取代际锁。 */
int generation_barrier(){
    SerialFixture serial;CHECK(serial.open());LiveOutputState state;
    std::mutex mutex;std::condition_variable changed;bool hold=false,entered=false;
    std::ostringstream errors;app::SerialCommandSink sink(serial.port,{},errors);
    CHECK(bind_test_output(sink,[&]() {
        {std::unique_lock<std::mutex> lock(mutex);if(hold){entered=true;changed.notify_all();changed.wait(lock,[&]{return !hold;});}}
        return state.snapshot();
    },[]{return runtime::monotonic_timestamp_ns();}));
    CHECK(sink.start());auto request=fresh_command(0.5,0);
    CHECK(sink.try_submit(request));CHECK(serial.wait_for(encoded(true,true,0,0.5)));
    bool observed=false;
    {std::unique_lock<std::mutex> lock(mutex);hold=true;observed=changed.wait_for(lock,500ms,[&]{return entered;});}
    if(!observed){{std::lock_guard<std::mutex> lock(mutex);hold=false;}changed.notify_all();}
    CHECK(observed);
    auto switched=std::async(std::launch::async,[&]{sink.set_active_generation(1);});
    const bool fenced=switched.wait_for(20ms)==std::future_status::timeout;
    {std::lock_guard<std::mutex> lock(mutex);hold=false;}changed.notify_all();
    CHECK(switched.wait_for(500ms)==std::future_status::ready);switched.get();CHECK(fenced);
    serial.drain();CHECK(serial.wait_for(encoded(false,false)));
    CHECK(!sink.try_submit(request));state.generation=1;
    request=fresh_command(0.2,0,1);CHECK(sink.try_submit(request));CHECK(serial.wait_for(encoded(true,true,0,0.2)));
    sink.cancel();CHECK(!sink.try_submit(request));sink.stop();serial.drain();
    const auto reads=state.reads.load();ControlFrame frame;CHECK(!serial.read_frame(frame,30ms));CHECK(state.reads==reads);
    return 0;
}
}
int main(){CHECK(dynamic_roll()==0);CHECK(unchanged_lease()==0);CHECK(generation_barrier()==0);return 0;}
