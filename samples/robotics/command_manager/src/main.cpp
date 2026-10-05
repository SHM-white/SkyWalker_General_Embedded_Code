#include "board_config.hpp"
#include "telemetry.hpp"
#include <robotics/command/receiver_sources.hpp>
#include "../../common/rc_controls.hpp"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(command_manager_bench, LOG_LEVEL_INF);

namespace {
using namespace skywalker;
static communication::AsyncUart::DmaBuffers rc_dma __nocache;
static communication::AsyncUart::DmaBuffers vision_dma __nocache;
static communication::AsyncUart::DmaBuffers referee_dma __nocache;
static communication::RemoteReceiver remote(bench::inputs.remote_uart, rc_dma, bench::inputs.remote);
static communication::vision::AbProtocol protocol(bench::inputs.protocol);
static communication::vision::VisionReceiver vision(bench::inputs.vision_uart, vision_dma, protocol,
                                                    bench::inputs.vision);
static communication::RefereeReceiver referee(bench::inputs.referee_uart, referee_dma, bench::inputs.referee_version,
                                              bench::inputs.referee_timeout_ms);
static robotics::RemoteSource remote_source(remote);
class ArmedRemoteSource final : public robotics::ICommandSource {
public:
    robotics::SourceRole role() const override {
        return robotics::SourceRole::Operator;
    }
    int start() override {
        return remote_source.start();
    }
    int sample(robotics::SourceSample &out) override {
        const int ret = remote_source.sample(out);
        if (ret != 0)
            return ret;
        auto &input = std::get<robotics::RemoteState>(out.value);
        const auto &state = controls_.update(input, k_uptime_get());
        if (!state.run_allowed)
            input.left_switch = robotics::RcSwitch::Down;
        return 0;
    }

private:
    samples::control::RcControlAdapter controls_{{true}};
} armed_remote_source;
static robotics::VisionSource vision_source(vision);
static robotics::RefereePermissionSource permission_source(referee);
static robotics::CommandManager manager(bench::manager);
static bench::Telemetry telemetry;
}

int main() {
    int ret = manager.registerSource(armed_remote_source);
    if (ret == 0)
        ret = manager.registerSource(vision_source);
    if (ret == 0)
        ret = manager.bindPermissions(permission_source);
    if (ret == 0)
        ret = manager.start();
    if (ret < 0) {
        LOG_ERR("command service start: %d", ret);
        return ret;
    }
    const int output = telemetry.start(bench::telemetry_uart);
    LOG_INF("command service started; telemetry=%d; command observation bench", output);
    robotics::CommandSnapshot frame{};
    for (;;) {
        if (manager.snapshot(frame) == 0)
            telemetry.emit(frame);
        k_sleep(K_MSEC(10));
    }
}
