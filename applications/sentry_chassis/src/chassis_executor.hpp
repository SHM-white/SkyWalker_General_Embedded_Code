#pragma once
#include <core/clock.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/chassis/chassis_power_limiter.hpp>
#include "board_config.hpp"
// Owns the complete chassis control/recovery sequence on one execution thread.
class ChassisExecutor {
public:
    explicit ChassisExecutor(skywalker::communication::InterBoardEndpoint &link) : link_(link) {}
    skywalker::robotics::RunStatus begin();
    skywalker::robotics::RunStatus update(skywalker::core::TimeUs now_us);
private:
    void suspend();
    skywalker::communication::InterBoardEndpoint &link_;
    DjiChassisHardware hardware_{board_config::motors};
    skywalker::robotics::SwerveChassis chassis_{board_config::chassisConfig()};
    skywalker::robotics::ChassisPowerLimiter limiter_{};
    skywalker::robotics::RunStatus status_{};
    bool checked_ = false, initialized_ = false, prepared_ = false, have_time_ = false;
    int config_error_ = 0;
    std::uint64_t peer_boot_ = 0, retry_ms_ = 0, ready_ms_ = 0;
    skywalker::core::TimeUs previous_us_ = 0;
    std::uint32_t generation_ = 0;
};
