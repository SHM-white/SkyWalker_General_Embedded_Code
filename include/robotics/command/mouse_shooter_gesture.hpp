#pragma once
#include <robotics/messages/remote.hpp>

namespace skywalker::robotics {
enum class MouseFeedKind : std::uint8_t { Idle, ClickSingle, RapidContinuous, HeldContinuous };
struct MouseFireIntent {
    bool friction_requested = false;
    MouseFeedKind kind = MouseFeedKind::Idle;
    std::uint32_t click_event_id = 0;
    MessageStamp click_stamp{};
    MessageStamp source_stamp{};
};

// 唯一命令线程拥有此长期对象；执行器只消费意图，不重新判断鼠标边沿。
class MouseShooterGesture {
public:
    struct Config {
        // 调参：用户指定的连点按下间隔；严格小于此值才进入连发，满此值退出。
        std::uint32_t rapid_click_gap_ms = 500;
        // 调参：建议初值，需无弹实机确认；与连点窗口独立。
        std::uint32_t hold_to_auto_ms = 250;
        // 调参：输入失联撤销门槛；仲裁器会统一使用 input_timeout_ms。
        std::uint32_t input_timeout_ms = 100;
    };
    explicit MouseShooterGesture(const Config &config) : config_(config) {
    }
    [[nodiscard]] int configError() const;
    // now_ms 检查年龄/超时；按下间隔和长按只使用新原始帧的时间戳。
    int update(const RemoteState &, bool keyboard_mouse_selected, bool run_allowed,
               std::uint64_t now_ms, MouseFireIntent &out);
    // 保留全局事件编号和已消费帧身份；恢复后必须先收到左键释放。
    void withdraw();

private:
    void clearFeed();
    void output(std::uint64_t now_ms, MouseFireIntent &out);
    const Config config_;
    MessageStamp last_frame_{}, source_stamp_{}, click_stamp_{};
    std::uint64_t press_ms_ = 0, last_click_ms_ = 0;
    std::uint32_t event_sequence_ = 0, pending_event_ = 0;
    MouseFeedKind kind_ = MouseFeedKind::Idle;
    bool selected_ = false, right_ = false, left_ = false;
    bool require_release_ = true, have_click_ = false, have_press_ = false;
};
}
