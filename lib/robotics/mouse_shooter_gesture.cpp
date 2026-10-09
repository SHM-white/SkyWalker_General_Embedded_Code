#include <cerrno>
#include <robotics/command/mouse_shooter_gesture.hpp>

namespace skywalker::robotics {
int MouseShooterGesture::configError() const {
    return config_.rapid_click_gap_ms && config_.hold_to_auto_ms && config_.input_timeout_ms ? 0 : -EINVAL;
}
void MouseShooterGesture::clearFeed() {
    pending_event_ = 0;
    click_stamp_ = {};
    kind_ = MouseFeedKind::Idle;
    have_click_ = have_press_ = false;
    press_ms_ = last_click_ms_ = 0;
}
void MouseShooterGesture::withdraw() {
    clearFeed();
    selected_ = right_ = left_ = false;
    source_stamp_ = {};
    require_release_ = true;
}
void MouseShooterGesture::output(std::uint64_t now, MouseFireIntent &out) {
    if (kind_ == MouseFeedKind::RapidContinuous &&
        (!have_click_ || now < last_click_ms_ || now - last_click_ms_ >= config_.rapid_click_gap_ms)) {
        clearFeed();
    }
    if (pending_event_ && !isFresh(click_stamp_, now, config_.input_timeout_ms)) {
        pending_event_ = 0;
        click_stamp_ = {};
        if (kind_ == MouseFeedKind::ClickSingle)
            kind_ = MouseFeedKind::Idle;
    }
    out = {};
    out.friction_requested = selected_ && right_;
    out.source_stamp = source_stamp_;
    if (!out.friction_requested)
        return;
    out.kind = kind_;
    out.click_event_id = pending_event_;
    out.click_stamp = click_stamp_;
}
int MouseShooterGesture::update(const RemoteState &r, bool selected, bool allowed,
                               std::uint64_t now, MouseFireIntent &out) {
    out = {};
    if (configError() < 0) {
        withdraw();
        return -EINVAL;
    }
    if (!r.online || !isFresh(r.stamp, now, config_.input_timeout_ms)) {
        withdraw();
        return -ESTALE;
    }
    const bool duplicate = last_frame_.valid && r.stamp.sequence == last_frame_.sequence &&
                           r.stamp.timestamp_ms == last_frame_.timestamp_ms;
    if (last_frame_.valid && !duplicate &&
        (!sequenceAfter(r.stamp.sequence, last_frame_.sequence) ||
         r.stamp.timestamp_ms < last_frame_.timestamp_ms)) {
        withdraw();
        // 接收会话重建也必须先释放；记录边界帧避免一直拒绝新会话的后续帧。
        last_frame_ = r.stamp;
        return -ESTALE;
    }
    if (!duplicate)
        last_frame_ = r.stamp;
    if (!selected || !allowed) {
        withdraw();
        return 0;
    }
    if (duplicate) {
        output(now, out);
        return 0;
    }
    source_stamp_ = r.stamp;
    const bool entering = !selected_;
    selected_ = true;
    if (!r.mouse.right) {
        clearFeed();
        right_ = false;
        left_ = r.mouse.left;
        require_release_ = true;
        output(now, out);
        return 0;
    }
    const bool right_entering = !right_;
    right_ = true;
    if (entering || right_entering) {
        clearFeed();
        require_release_ = true;
    }
    if (require_release_) {
        left_ = r.mouse.left;
        if (!left_)
            require_release_ = false;
        output(now, out);
        return 0;
    }
    const auto input_time = r.stamp.timestamp_ms;
    if (r.mouse.left && !left_) {
        const bool rapid = have_click_ && input_time >= last_click_ms_ &&
                           input_time - last_click_ms_ < config_.rapid_click_gap_ms;
        press_ms_ = last_click_ms_ = input_time;
        have_click_ = have_press_ = true;
        if (rapid) {
            pending_event_ = 0;
            click_stamp_ = {};
            kind_ = MouseFeedKind::RapidContinuous;
        }
        else {
            if (++event_sequence_ == 0)
                ++event_sequence_;
            pending_event_ = event_sequence_;
            click_stamp_ = r.stamp;
            kind_ = MouseFeedKind::ClickSingle;
        }
    }
    if (r.mouse.left && have_press_ && input_time >= press_ms_ &&
        input_time - press_ms_ >= config_.hold_to_auto_ms) {
        kind_ = MouseFeedKind::HeldContinuous;
        pending_event_ = 0;
        click_stamp_ = {};
    }
    if (!r.mouse.left && left_) {
        have_press_ = false;
        if (kind_ == MouseFeedKind::HeldContinuous)
            clearFeed();
    }
    left_ = r.mouse.left;
    output(now, out);
    return 0;
}
}
