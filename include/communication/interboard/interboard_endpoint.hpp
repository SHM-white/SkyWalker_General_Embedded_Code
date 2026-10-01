#pragma once
#include <communication/async_uart.hpp>
#include <communication/interboard/interboard_link.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/messages/referee.hpp>
namespace skywalker::communication {
// poll() has one communication-thread owner. The other methods exchange copies
// under a short lock. Endpoint and external DMA storage have static lifetime.
class InterBoardEndpoint {
public:
    struct Config {
        robotics::BoardRole role;
        std::uint32_t command_timeout_ms = 100, heartbeat_timeout_ms = 100;
    };
    struct Snapshot {
        robotics::BoardHeartbeat peer{};
        robotics::RemoteChassisControl control{};
        robotics::ChassisConstraint constraint{};
        robotics::ChassisFeedbackSummary feedback{};
        std::uint64_t local_boot_id = 0;
        bool online = false;
        int error = 0;
    };
    InterBoardEndpoint(const device *uart, AsyncUart::DmaBuffers &dma, const Config &config)
        : config_(config), uart_(uart, dma), link_(config.role) {}
    void submit(const robotics::ChassisCommand &);
    void setReferee(const robotics::RefereeState &);
    void setStatus(const robotics::RunStatus &);
    Snapshot snapshot() const;
    void poll(std::uint64_t now_ms);
private:
    struct Outgoing {
        robotics::ChassisCommand command{};
        robotics::RefereeState referee{};
        robotics::RunStatus status{};
    };
    Config config_;
    AsyncUart uart_;
    InterBoardLink link_;
    mutable k_spinlock lock_{};
    Outgoing outgoing_{};
    Snapshot published_{};
    std::uint64_t boot_id_ = 0, retry_ms_ = 0, next_tx_ms_ = 0, next_heartbeat_ms_ = 0;
    std::uint64_t peer_boot_ = 0;
    std::uint32_t peer_generation_ = 0, baseline_sequence_ = 0;
    std::uint32_t heartbeat_sequence_ = 0, control_sequence_ = 0, constraint_sequence_ = 0, feedback_sequence_ = 0;
    bool peer_online_ = false, have_baseline_ = false;
    int error_ = 0;
};
}
