// TODO(legacy): historical private implementation. The active entry now uses
// samples/robotics/common/vehicle_bench.hpp and central vehicle calibration.
#pragma once
#include <core/clock.hpp>
#include <robotics/execution/run_status.hpp>
#include "board_config.hpp"
// Single execution-thread owner; this object owns the CAN controller and axis.
class GimbalExecutor {
public:
    skywalker::robotics::RunStatus begin();
    skywalker::robotics::RunStatus update(const skywalker::robotics::GimbalCommand &, skywalker::core::TimeUs now_us);
private:
    void suspend();
    skywalker::motor::CanBus bus_{board_config::yaw_can};
    skywalker::motor::Motor drive_{board_config::yawMotorConfig()};
    skywalker::robotics::GimbalAxis axis_{drive_, board_config::motorConfig(drive_.info()), board_config::yaw};
    skywalker::robotics::RunStatus status_{};
    bool checked_ = false, attached_ = false, started_ = false, configured_ = false, have_time_ = false;
    int config_error_ = 0;
    skywalker::core::TimeUs previous_us_ = 0;
    std::uint64_t retry_ms_ = 0;
};
