#pragma once
#include <robotics/messages/remote.hpp>
namespace skywalker::robotics {
class ManualCommandMapper {
public:
    struct Config {
        float analog_deadband = 0.03f, mouse_yaw_scale = 0.002f, mouse_pitch_scale = 0.002f;
        float channel_range = 660;
    };
    explicit ManualCommandMapper(const Config &config) : config_(config) {
    }
    int map(const RemoteState &remote, OperatorIntent &out) const;

private:
    Config config_;
};
}
