#pragma once
#include <array>
#include <communication/interboard/interboard_transport.hpp>
#include <communication/interboard/interboard_link.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/messages/referee.hpp>
#include <zephyr/spinlock.h>
namespace skywalker::communication {
// poll() has one communication-thread owner. Other calls exchange owned copies
// under a short lock. Endpoint, transport and callback storage live until reboot.
class InterBoardEndpoint {
public:
    struct Config {
        robotics::BoardRole role;
        std::uint32_t command_timeout_ms = 100, heartbeat_timeout_ms = 100;
        std::uint32_t tx_timeout_ms = 60;
        std::uint32_t status_timeout_ms = 100;
    };
    struct Snapshot {
        robotics::BoardHeartbeat peer{};
        robotics::RemoteChassisControl control{};
        robotics::ChassisConstraint constraint{};
        robotics::ChassisFeedbackSummary feedback{};
        robotics::OperatorControl operator_control{};
        robotics::BigYawRequest big_yaw_request{};
        robotics::BigYawFeedback big_yaw_feedback{};
        bool operator_control_valid = false;
        std::uint64_t local_boot_id = 0;
        bool online = false;
        int error = 0;
        InterBoardTransportKind transport = InterBoardTransportKind::Uart;
        InterBoardParser::Stats parser_stats{};
        std::uint32_t rejected_frames = 0;
    };
    InterBoardEndpoint(InterBoardTransport &transport, const Config &config)
        : config_(config), transport_(transport), link_(config.role) {}
    InterBoardEndpoint(const InterBoardEndpoint &) = delete;
    InterBoardEndpoint &operator=(const InterBoardEndpoint &) = delete;
    void submit(const robotics::ChassisCommand &);
    // Thread-only. 0 copies; -EACCES for a non-gimbal producer; -EINVAL for
    // contradictory run/stop/clear requests. Ages are measured at value.stamp.
    // receiver_boot_id is the producer's binding, including for clear retries.
    [[nodiscard]] int submitOperatorControl(const robotics::OperatorControl &);
    void setReferee(const robotics::RefereeState &);
    void setStatus(const robotics::RunStatus &);
    void submitBigYaw(const robotics::BigYawRequest &);
    void setBigYawFeedback(const robotics::BigYawFeedback &);
    Snapshot snapshot() const;
    void poll(std::uint64_t now_ms);
private:
    struct Outgoing {
        robotics::ChassisCommand command{};
        robotics::RefereeState referee{};
        robotics::RunStatus status{};
        robotics::OperatorControl operator_control{};
        robotics::BigYawRequest big_yaw_request{};
        robotics::BigYawFeedback big_yaw_feedback{};
    };
    Config config_;
    InterBoardTransport &transport_;
    InterBoardLink link_;
    mutable k_spinlock lock_{};
    Outgoing outgoing_{};
    Snapshot published_{};
    std::uint64_t boot_id_ = 0, next_tx_ms_ = 0;
    std::array<std::uint64_t, kMessageCount> next_message_ms_{};
    std::array<std::uint32_t, kMessageCount> wire_sequence_{};
    std::uint64_t peer_boot_ = 0;
    std::uint32_t peer_generation_ = 0, baseline_sequence_ = 0;
    bool peer_online_ = false, have_baseline_ = false;
    bool big_yaw_context_valid_ = false, have_big_yaw_baseline_ = false;
    std::uint64_t big_yaw_peer_boot_ = 0;
    std::uint32_t big_yaw_peer_generation_ = 0, big_yaw_baseline_sequence_ = 0;
    int error_ = 0;
};
}
