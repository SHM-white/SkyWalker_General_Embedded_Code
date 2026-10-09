#pragma once
#include <array>
#include <communication/remote/dr16_decoder.hpp>
#include <zephyr/kernel.h>
namespace skywalker::communication {
// A single input task owns framing and snapshots. UART callbacks only enqueue bytes.
class RemoteService {
public:
    struct Config {
        std::uint32_t offline_timeout_ms = 100, assembly_gap_ms = 10;
    };
    RemoteService(const Dr16Decoder::Config &d, const Config &c) : decoder_(d), config_(c) {
    }
    int processBytes(const std::uint8_t *, std::size_t, std::uint64_t now_ms);
    int snapshot(std::uint64_t now_ms, robotics::RemoteState &out) const;
    // 唯一命令线程消费；诊断 snapshot 不消费。返回 0/-EAGAIN/-EOVERFLOW/-ESTALE。
    int nextFrame(std::uint64_t now_ms, robotics::RemoteState &out);
    bool online(std::uint64_t now_ms) const;
    void discardPartial() {
        used_ = 0;
        invalidateFrames();
    }

private:
    Dr16Decoder decoder_;
    Config config_;
    robotics::RemoteState latest_{};
    std::array<std::uint8_t, 18> buffer_{};
    std::size_t used_ = 0;
    std::uint64_t last_bytes_ms_ = 0;
    void publishFrame(const robotics::RemoteState &);
    void invalidateFrames();
    // 调参：队列只覆盖短调度停顿；不可通过扩大它延长输入有效期。
    static constexpr std::size_t kFrameCapacity = 32;
    std::array<robotics::RemoteState, kFrameCapacity> frames_{};
    std::size_t frame_head_ = 0, frame_count_ = 0;
    bool frame_loss_ = false;
    k_spinlock frame_lock_{};
};
}
