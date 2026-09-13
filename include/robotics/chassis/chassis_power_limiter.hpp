#pragma once
namespace skywalker::robotics {
struct ChassisPowerInput {
    float measured_power_w = 0, power_limit_w = 0, buffer_energy_j = 0;
};
struct ChassisPowerDecision {
    float effort_scale = 1;
};
// Bench heuristic only; does not certify compliance with a competition power limit.
class ChassisPowerLimiter {
public:
    struct Config {
        float recovery_per_s = 0.5f, buffer_reserve_j = 10;
    };
    ChassisPowerLimiter() = default;
    explicit ChassisPowerLimiter(const Config &config) : config_(config) {
    }
    int reset();
    int step(const ChassisPowerInput &, float dt_s, ChassisPowerDecision &out);

private:
    Config config_{};
    float scale_ = 0;
};
}
