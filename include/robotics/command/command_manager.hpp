#pragma once
#include <robotics/messages/command.hpp>
#include <robotics/messages/remote.hpp>
#include <robotics/messages/safety.hpp>
namespace skywalker::robotics {
class CommandManager {
public:
    struct Config {
        float max_chassis_vx_m_s = 3, max_chassis_vy_m_s = 3, max_chassis_wz_rad_s = 6;
        float max_gimbal_yaw_rate_rad_s = 3, max_gimbal_pitch_rate_rad_s = 2;
        std::uint32_t input_timeout_ms = 100;
    };
    explicit CommandManager(const Config &config) : config_(config) {
    }
    int reset(std::uint64_t now_ms);
    int step(const OperatorIntent &, const GlobalSafetyDecision &, std::uint64_t now_ms, RobotCommand &out);

private:
    Config config_;
    std::uint32_t sequence_ = 0;
};
}
