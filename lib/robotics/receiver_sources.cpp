#include <cerrno>
#include <algorithm>
#include <core/clock.hpp>
#include <robotics/command/receiver_sources.hpp>
namespace skywalker::robotics {
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_INTERBOARD_ENDPOINT)
int InterBoardRefereeSource::sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics) {
    const auto peer = endpoint_.snapshot();
    diagnostics = {};
    diagnostics.error = peer.error;
    if (!peer.online || peer.peer.sender_boot_id != peer_boot_) {
        service_.reset();
        peer_boot_ = peer.online ? peer.peer.sender_boot_id : 0;
    }
    for (unsigned budget = 0; budget < 8; ++budget) {
        communication::RefereeRelayFrame frame{};
        const int result = endpoint_.readRefereeFrame(frame);
        if (result == -EAGAIN)
            break;
        if (result < 0) {
            diagnostics.error = result;
            break;
        }
        if (frame.source_age_ms > frame.receive_ms)
            continue;
        // Convert the producer's age into this board's time domain. Polling
        // the cache later must never renew a referee permission's source time.
        service_.processBytes(frame.bytes.data(), frame.size, frame.receive_ms - frame.source_age_ms);
    }
    now_ms = std::max(now_ms, core::monotonicTimeUs() / 1000);
    service_.processBytes(nullptr, 0, now_ms);
    out = {};
    if (peer.online)
        service_.snapshot(now_ms, out);
    return 0;
}
#endif
#ifdef CONFIG_SKYWALKER_NAVIGATION_RECEIVER
int NavigationSource::start() {
    return receiver_.start();
}
int NavigationSource::sample(SourceSample &out) {
    const auto value = receiver_.snapshot();
    out.value = value.protocol.command;
    out.diagnostics = {};
    out.diagnostics.error = value.uart_error;
    out.diagnostics.state = static_cast<std::uint32_t>(value.state);
    out.diagnostics.dropped = value.dropped;
    out.diagnostics.dropped_available = true;
    return 0;
}
#endif
#ifdef CONFIG_SKYWALKER_REMOTE_RECEIVER
int RemoteSource::start() {
    return receiver_.start();
}
int RemoteSource::nextOperatorFrame(RemoteState &out) {
    return receiver_.nextFrame(out);
}
int RemoteSource::sample(SourceSample &out) {
    const int ret = receiver_.snapshot(cached_);
    out.value = cached_.remote;
    out.diagnostics = {};
    out.diagnostics.error = cached_.uart_error;
    out.diagnostics.sample_error = ret;
    out.diagnostics.state = static_cast<std::uint32_t>(cached_.state);
    out.diagnostics.dropped = cached_.dropped;
    out.diagnostics.dropped_available = true;
    // Even on contention RemoteReceiver expires the retained online flag.
    return ret == -EAGAIN ? 0 : ret;
}
#endif
#ifdef CONFIG_SKYWALKER_VISION_RECEIVER
int VisionSource::start() {
    return receiver_.start();
}
int VisionSource::sample(SourceSample &out) {
    const auto value = receiver_.snapshot();
    out.value = value.link.aim;
    out.diagnostics = {};
    out.diagnostics.error = value.uart_error;
    out.diagnostics.state = static_cast<std::uint32_t>(value.state);
    out.diagnostics.dropped = value.dropped;
    out.diagnostics.dropped_available = true;
    return 0;
}
#endif
#if defined(CONFIG_SKYWALKER_REFEREE) && defined(CONFIG_SKYWALKER_UART_TRANSPORT)
int RefereePermissionSource::start() {
    return 0;
}
int RefereePermissionSource::sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics) {
    out = receiver_.poll(now_ms);
    diagnostics = {};
    diagnostics.error = receiver_.error();
    // RefereeReceiver does not expose transport drop counters.
    return 0;
}
#endif
}
