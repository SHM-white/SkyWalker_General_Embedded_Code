#include <cerrno>
#include <robotics/command/receiver_sources.hpp>
namespace skywalker::robotics {
#ifdef CONFIG_SKYWALKER_REMOTE_RECEIVER
int RemoteSource::start() { return receiver_.start(); }
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
int VisionSource::start() { return receiver_.start(); }
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
int RefereePermissionSource::start() { return 0; }
int RefereePermissionSource::sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics) {
    out = receiver_.poll(now_ms);
    diagnostics = {};
    diagnostics.error = receiver_.error();
    // RefereeReceiver does not expose transport drop counters.
    return 0;
}
#endif
}
