#pragma once
#include <robotics/command/command_source.hpp>
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_INTERBOARD_ENDPOINT)
#include <communication/interboard/interboard_endpoint.hpp>
#include <communication/referee/referee_service.hpp>
#endif
#ifdef CONFIG_SKYWALKER_NAVIGATION_RECEIVER
#include <communication/navigation/navigation_receiver.hpp>
#endif
#ifdef CONFIG_SKYWALKER_REMOTE_RECEIVER
#include <communication/remote/remote_receiver.hpp>
#endif
#ifdef CONFIG_SKYWALKER_VISION_RECEIVER
#include <communication/vision/vision_receiver.hpp>
#endif
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_UART_TRANSPORT)
#include <communication/referee/referee_receiver.hpp>
#endif

namespace skywalker::robotics {
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_INTERBOARD_ENDPOINT)
// CommandManager consumes reassembled frames and owns the referee decoder.
// No local UART or shared parser is involved on the gimbal board.
class InterBoardRefereeSource final : public IPermissionSource {
public:
    explicit InterBoardRefereeSource(communication::InterBoardEndpoint &endpoint) : endpoint_(endpoint) {
    }
    int start() override {
        return 0;
    }
    int sample(std::uint64_t now_ms, RefereeState &, SourceDiagnostics &) override;
    void setFrameHandler(communication::RefereeParser::FrameHandler handler, void *context) {
        service_.setFrameHandler(handler, context);
    }

private:
    communication::InterBoardEndpoint &endpoint_;
    communication::RefereeService service_;
    std::uint64_t peer_boot_ = 0;
};
#endif
#ifdef CONFIG_SKYWALKER_NAVIGATION_RECEIVER
class NavigationSource final : public ICommandSource {
public:
    explicit NavigationSource(communication::navigation::NavigationReceiver &receiver) : receiver_(receiver) {
    }
    SourceRole role() const override {
        return SourceRole::Navigation;
    }
    int start() override;
    int sample(SourceSample &out) override;

private:
    communication::navigation::NavigationReceiver &receiver_;
};
#endif
#ifdef CONFIG_SKYWALKER_REMOTE_RECEIVER
class RemoteSource final : public ICommandSource {
public:
    explicit RemoteSource(communication::RemoteReceiver &receiver) : receiver_(receiver) {
    }
    SourceRole role() const override {
        return SourceRole::Operator;
    }
    int start() override;
    int sample(SourceSample &out) override;
    int nextOperatorFrame(RemoteState &out) override;

private:
    communication::RemoteReceiver &receiver_;
    communication::RemoteReceiver::Snapshot cached_{};
};
#endif
#ifdef CONFIG_SKYWALKER_VISION_RECEIVER
class VisionSource final : public ICommandSource {
public:
    explicit VisionSource(communication::vision::VisionReceiver &receiver) : receiver_(receiver) {
    }
    SourceRole role() const override {
        return SourceRole::Aim;
    }
    int start() override;
    int sample(SourceSample &out) override;

private:
    communication::vision::VisionReceiver &receiver_;
};
#endif
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_UART_TRANSPORT)
class RefereePermissionSource final : public IPermissionSource {
public:
    explicit RefereePermissionSource(communication::RefereeReceiver &receiver) : receiver_(receiver) {
    }
    int start() override;
    int sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics) override;

private:
    communication::RefereeReceiver &receiver_;
};
#endif
}
