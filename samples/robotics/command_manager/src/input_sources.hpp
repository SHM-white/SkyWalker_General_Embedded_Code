#pragma once
#include <communication/remote/remote_receiver.hpp>
#include <communication/vision/ab_protocol.hpp>
#include <communication/vision/vision_receiver.hpp>
#include <communication/referee/referee_service.hpp>
#include <robotics/command/command_inputs.hpp>

namespace bench {
using namespace skywalker;
namespace vision = communication::vision;

struct InputFrame {
    robotics::CommandInputs commands{};
    communication::RemoteReceiver::State remote_state{};
    vision::VisionReceiver::State vision_state{};
    int remote_error = 0, vision_error = 0, referee_error = 0;
    std::uint32_t remote_dropped = 0, vision_dropped = 0, referee_dropped = 0;
    std::uint32_t referee_resets = 0;
};

class InputSources {
public:
    struct Config {
        const device *remote_uart = nullptr, *vision_uart = nullptr, *referee_uart = nullptr;
        communication::RemoteReceiver::Config remote{};
        vision::AbProtocol::Config protocol{};
        vision::VisionReceiver::Config vision{};
        communication::RefereeVersion referee_version = communication::RefereeVersion::Rm2026V1_3;
        std::uint32_t referee_timeout_ms = 500;
    };

    InputSources(const Config &c, communication::AsyncUart::DmaBuffers &r, communication::AsyncUart::DmaBuffers &v,
                 communication::AsyncUart::DmaBuffers &f)
        : remote_(c.remote_uart, r, c.remote), protocol_(c.protocol), vision_(c.vision_uart, v, protocol_, c.vision),
          referee_uart_(c.referee_uart, f), referee_(c.referee_version, c.referee_timeout_ms) {
    }
    [[nodiscard]] int start();
    InputFrame poll();

private:
    communication::RemoteReceiver remote_;
    vision::AbProtocol protocol_; // 必须在 vision_ 之前构造并持续存活。
    vision::VisionReceiver vision_;
    communication::AsyncUart referee_uart_;
    communication::RefereeService referee_;
    communication::RemoteReceiver::Snapshot rc_{}; // 长期保留，不能每轮清零。
    int remote_start_error_ = 0, vision_start_error_ = 0, referee_error_ = 0;
    std::uint32_t referee_resets_ = 0;
    std::uint64_t referee_init_retry_ms_ = 0;
    bool started_ = false;
};
}
