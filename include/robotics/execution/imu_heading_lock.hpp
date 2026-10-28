#pragma once
#include <robotics/messages/interboard.hpp>
namespace skywalker::robotics {
// Output is joint torque (N m). MIT current/torque staging stays in Motor.
class ImuHeadingLock {
public:
    struct Config {
        float heading_kp=2, correction_rate_max_rad_s=1.5f, error_max_rad=1.5707963f;
        float rate_kp_nm_s=.02f, rate_ki_nm=.15f, torque_max_nm=.5f;
    };
    explicit ImuHeadingLock(const Config &c) : config_(c) {}
    int validate() const;
    void reset();
    int step(const GimbalImuYaw &, float operator_rate_rad_s, float dt_s, std::uint64_t now_ms, float &torque_nm);
    float targetRad() const { return target_; }
private:
    Config config_;
    float target_=0, integral_=0, previous_rate_=0;
    std::uint64_t boot_=0;
    std::uint32_t generation_=0;
    bool initialized_=false;
};
}
