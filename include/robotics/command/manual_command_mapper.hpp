#pragma once
#include <robotics/messages/remote.hpp>
namespace skywalker::robotics {
// ShooterSelectable 专用于台架：左 Down=Safe、Middle=RC、Up=键鼠；保留其他配置的 Auto 含义。
enum class RemoteInputProfile : std::uint8_t { PhysicalRemote, KeyboardMouseSelectable, ShooterSelectable };
class ManualCommandMapper {
public:
    struct Config {
        float analog_deadband = 0.03f;
        // 旧输入配置的归一化兼容比例；台架改用下方独立速度灵敏度。
        float mouse_yaw_scale = 0.002f, mouse_pitch_scale = 0.002f;
        // 调参：仅 ShooterSelectable 使用，单位 (rad/s)/原始透传单位。
        // 这是线性速度合同，不假定 raw 值为硬件 counts；与机构 max_rate 独立。
        float mouse_yaw_rate_per_unit = 0.002f, mouse_pitch_rate_per_unit = 0.0016f;
        float channel_range = 660;
        RemoteInputProfile input_profile = RemoteInputProfile::KeyboardMouseSelectable;
    };
    explicit ManualCommandMapper(const Config &config) : config_(config) {
    }
    int map(const RemoteState &remote, OperatorIntent &out) const;

private:
    Config config_;
};
}
