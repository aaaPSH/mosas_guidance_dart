#include "serial_test_support.hpp"
#include "test_support.hpp"
#include <sstream>
#include <thread>
namespace {
using namespace mosas;
using namespace mosas::test;
using namespace std::chrono_literals;
int lifecycle(){
    SerialFixture serial;CHECK(serial.open());LiveOutputState state;std::ostringstream errors;
    app::SerialCommandSink sink(serial.port,{},errors);
    CHECK(!sink.start()); // 未绑定不能启用旧 body 命令路径。
    CHECK(bind_live_output(sink,state));CHECK(sink.start());
    ControlFrame frame{};CHECK(serial.read_frame(frame));CHECK(frame==encoded(false,false));
    auto command=fresh_command();const auto submitted=SerialClock::now();
    CHECK(sink.try_submit(command));CHECK(serial.wait_for(encoded(true,true,-0.11,0.23)));
    for(int i=0;i<2;++i){CHECK(serial.read_frame(frame));CHECK(frame==encoded(true,true,-0.11,0.23));}
    CHECK(serial.wait_for(encoded(true,false),250ms));CHECK(SerialClock::now()-submitted>=80ms);
    command.valid=false;command.invalid_reason=domain::GuidanceInvalidReason::invalid_rate;
    CHECK(sink.try_submit(command));CHECK(serial.wait_for(encoded(true,true)));
    command.valid=true;CHECK(sink.try_submit(command));CHECK(serial.wait_for(encoded(true,true,-0.11,0.23)));
    state.generation=1;sink.set_active_generation(1);serial.drain();CHECK(serial.read_frame(frame));CHECK(frame==encoded(false,false));
    CHECK(!sink.try_submit(command));
    command.generation=1;command.launched=false;
    CHECK(sink.try_submit(command));CHECK(serial.wait_for(encoded(false,false)));
    command.launched=true;CHECK(sink.try_submit(command));CHECK(serial.wait_for(encoded(true,true,-0.11,0.23)));
    sink.cancel();CHECK(!sink.try_submit(command));sink.stop();serial.drain();
    const auto reads=state.reads.load();CHECK(!serial.read_frame(frame,30ms));CHECK(state.reads==reads);
    CHECK(sink.start());CHECK(serial.read_frame(frame));CHECK(frame==encoded(false,false));sink.stop();
    CHECK(sink.diagnostics().send_failure_count==0);
    return 0;
}
int sampling(){
    SerialFixture serial;CHECK(serial.open());LiveOutputState state;std::ostringstream errors;
    app::SerialCommandSink sink(serial.port,{},errors);CHECK(bind_live_output(sink,state));CHECK(sink.start());
    auto command=fresh_command();const auto started=SerialClock::now();
    for(int i=1;i<=36;++i){
        command.overload_g={0,i*0.01,0};command.source_frame_id=i;
        CHECK(sink.try_submit(command));
        std::this_thread::sleep_until(started+std::chrono::nanoseconds(static_cast<std::int64_t>(i)*1'000'000'000/120));
    }
    sink.stop();const auto frames=serial.drain();CHECK(frames.size()>=20&&frames.size()<=34);
    bool skipped=false;int previous=0;
    for(const auto& frame:frames){
        if(frame==encoded(false,false))continue;
        const int sequence=frame[6]|(static_cast<int>(frame[7])<<8);
        CHECK(sequence>=previous&&sequence>=1&&sequence<=36);
        CHECK(frame==encoded(true,true,0,sequence*0.01));
        skipped=skipped||sequence>previous+1;previous=sequence;
    }
    CHECK(skipped&&previous>=33);CHECK(errors.str().empty());return 0;
}
int cadence(){
    SerialFixture serial;CHECK(serial.open());LiveOutputState state;std::ostringstream errors;
    app::CommandAppConfig config;config.send_rate_hz=10;config.command_timeout_ms=1;
    app::SerialCommandSink sink(serial.port,config,errors);CHECK(bind_live_output(sink,state));CHECK(sink.start());
    ControlFrame frame{};CHECK(serial.read_frame(frame));CHECK(frame==encoded(false,false));
    const auto submitted=SerialClock::now();CHECK(sink.try_submit(fresh_command()));
    CHECK(serial.read_frame(frame));CHECK(frame==encoded(true,false));CHECK(SerialClock::now()-submitted>=60ms);
    sink.stop();serial.drain();
    config.send_rate_hz=1;
    app::SerialCommandSink slow(serial.port,config,errors);CHECK(bind_live_output(slow,state));CHECK(slow.start());CHECK(serial.read_frame(frame));
    const auto stopped=SerialClock::now();slow.stop();CHECK(SerialClock::now()-stopped<200ms);
    config.send_rate_hz=0;
    app::SerialCommandSink invalid(serial.port,config,errors);CHECK(bind_live_output(invalid,state));CHECK(!invalid.start());
    return 0;
}
}
int main(){CHECK(lifecycle()==0);CHECK(sampling()==0);CHECK(cadence()==0);return 0;}
