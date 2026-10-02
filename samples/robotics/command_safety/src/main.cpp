#include <zephyr/logging/log.h>
#include <robotics/command/command_manager.hpp>
#include <robotics/command/receiver_sources.hpp>
#include "board_config.hpp"
#include "../../common/rc_controls.hpp"
LOG_MODULE_REGISTER(command_insurance, LOG_LEVEL_INF);
int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;
    static communication::AsyncUart::DmaBuffers dma __nocache;
    static communication::RemoteReceiver receiver(bench::remote_uart, dma, samples::control::receiverConfig());
    static RemoteSource source(receiver);
    class ArmedRemoteSource final : public ICommandSource {
    public:
        explicit ArmedRemoteSource(RemoteSource &source) : source_(source) {}
        SourceRole role() const override { return SourceRole::Operator; }
        int start() override { return source_.start(); }
        int sample(SourceSample &out) override {
            const int ret = source_.sample(out);
            if (ret != 0) return ret;
            auto &input = std::get<RemoteState>(out.value);
            const auto &state = controls_.update(input, k_uptime_get());
            if (!state.run_allowed) input.left_switch = RcSwitch::Down;
            return 0;
        }
    private:
        RemoteSource &source_;
        samples::control::RcControlAdapter controls_;
    };
    static ArmedRemoteSource armed_source(source);
    static const CommandManager::Config config = [] {
        CommandManager::Config value{};
        value.require_referee_for_motion = false;
        value.allow_auto = false;
        value.mapper.input_profile = RemoteInputProfile::PhysicalRemote;
        return value;
    }();
    static CommandManager manager(config);
    int ret = manager.registerSource(armed_source);
    if (ret == 0) ret = manager.start();
    if (ret < 0) { LOG_ERR("command service start: %d", ret); return ret; }
    CommandSnapshot frame{};
    std::uint64_t next_log = 0;
    LOG_INF("Motor-free RC bench: Down/Down centered 500ms, then left Middle to arm; Down stops");
    for (;;) {
        const auto now_ms = static_cast<std::uint64_t>(k_uptime_get());
        if (manager.snapshot(frame) == 0 && now_ms >= next_log) {
            next_log = now_ms + 100;
            const auto &decision = frame.decision;
            LOG_INF("online=%d mode=%u chassis=%u gimbal=%u reasons=%x error=%d seq=%u",
                    frame.observed.remote.online, unsigned(decision.operator_mode), unsigned(decision.command.chassis.mode),
                    unsigned(decision.command.gimbal.mode), decision.reasons(), decision.error, decision.command.stamp.sequence);
        }
        k_sleep(K_MSEC(10));
    }
}
