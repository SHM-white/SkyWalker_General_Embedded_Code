#pragma once
#include <algorithm>
#include <cmath>
#include <robotics/messages/remote.hpp>
namespace skywalker::robotics {
// ShooterSelectable 专用于台架：左 Down=Safe、左 Middle/Up=Manual，右 Middle=键鼠；
// 左 Middle 的导航档由 RcControlAdapter 的运动授权门控。其他配置保留 Auto 含义。
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
        // 视觉执行样例（allow_auto）把左 Up 保留给自动；其余样例左 Up 为手动档。
        bool auto_on_up = false;
        bool navigation_on_middle = false;
        // 手动档始终合并键鼠：不再要求右拨杆选择键鼠来源。每轴独立，
        // 物理摇杆离开死区时该轴听摇杆，回中时交给键盘/鼠标。
        bool merge_keyboard_mouse = false;
        // 左 Middle 时由右拨杆选择巡航子模式；默认关闭以保持旧样例映射。
        bool cruise_on_middle = false;
    };
    explicit ManualCommandMapper(const Config &config) : config_(config) {
    }
    // 单通道归一化：先按 range 缩放并钳制，再按死区线性重映射，死区内输出精确 0。
    // 调用方保证 range>0、0<=deadband<1 且参数有限（map() 在入口统一校验）。
    static float normalizeChannel(float raw, float range, float deadband) {
        const float x = std::clamp(raw / range, -1.0f, 1.0f);
        return std::fabs(x) <= deadband ? 0.0f : std::copysign((std::fabs(x) - deadband) / (1.0f - deadband), x);
    }
    int map(const RemoteState &remote, OperatorIntent &out) const;

private:
    Config config_;
};
}
