#include "input_sources.hpp"
#include <cerrno>
#include <zephyr/kernel.h>

namespace bench {
int InputSources::start() {
    if (started_)
        return -EALREADY;
    started_ = true;
    remote_start_error_ = remote_.start();
    vision_start_error_ = vision_.start();
    referee_error_ = referee_uart_.init();
    if (remote_start_error_ < 0)
        return remote_start_error_;
    if (vision_start_error_ < 0)
        return vision_start_error_;
    return referee_error_;
}

InputFrame InputSources::poll() {
    InputFrame frame{};
    auto now_ms = static_cast<std::uint64_t>(k_uptime_get());
    int sr = referee_uart_.service(now_ms);
    if (sr == -EACCES && now_ms >= referee_init_retry_ms_) {
        referee_init_retry_ms_ = now_ms + 100;
        sr = referee_uart_.init();
    }
    if (sr == 0)
        referee_error_ = 0;
    else if (sr != -EAGAIN && (sr != -EACCES || referee_error_ == 0))
        referee_error_ = sr;

    communication::AsyncUart::RxChunk chunk{};
    for (unsigned budget = 0; budget < 8; ++budget) {
        const int rr = referee_uart_.read(chunk);
        if (rr == -EOVERFLOW) {
            referee_.discardPartial();
            ++referee_resets_;
            continue;
        }
        if (rr == -EAGAIN)
            break;
        if (rr < 0) {
            if (rr != -EACCES || referee_error_ == 0)
                referee_error_ = rr;
            break;
        }
        const int pr = referee_.processBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
        if (pr < 0)
            referee_error_ = pr;
    }
    now_ms = static_cast<std::uint64_t>(k_uptime_get());
    referee_.processBytes(nullptr, 0, now_ms);
    // result 已清零；-EAGAIN 不写，-ESTALE 则写旧值并标 online=false。
    const int fs = referee_.snapshot(now_ms, frame.commands.referee);
    if (fs != 0 && fs != -EAGAIN && fs != -ESTALE)
        referee_error_ = fs;

    (void)remote_.snapshot(rc_);
    const auto vs = vision_.snapshot();
    frame.commands.remote = rc_.remote;
    frame.commands.vision = vs.link.aim;
    frame.remote_state = rc_.state;
    frame.vision_state = vs.state;
    frame.remote_error = remote_start_error_ < 0 ? remote_start_error_ : rc_.uart_error;
    frame.vision_error = vision_start_error_ < 0 ? vision_start_error_ : vs.uart_error;
    frame.referee_error = referee_error_;
    frame.remote_dropped = rc_.dropped;
    frame.vision_dropped = vs.dropped;
    frame.referee_dropped = referee_uart_.droppedChunks();
    frame.referee_resets = referee_resets_;
    return frame;
}
}
