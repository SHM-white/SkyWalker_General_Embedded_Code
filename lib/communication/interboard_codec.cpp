#include <cerrno>
#include <cmath>
#include <cstring>
#include <communication/wire.hpp>
#include <communication/interboard/interboard_codec.hpp>
namespace skywalker::communication {
using namespace robotics;
using namespace wire;
namespace {
int frame(BoardRole role, MessageId id, std::uint32_t seq, const std::uint8_t *payload, std::size_t n,
          std::uint8_t *out, std::size_t cap) {
    if (!out)
        return -EINVAL;
    if (cap < n + 14)
        return -ENOSPC;
    std::uint8_t tmp[kMaxFrame]{};
    tmp[0] = 0xa5;
    tmp[1] = 0x5a;
    tmp[2] = 1;
    tmp[3] = static_cast<std::uint8_t>(role);
    storeLe16(tmp + 4, static_cast<std::uint16_t>(id));
    storeLe16(tmp + 6, n);
    storeLe32(tmp + 8, seq);
    std::memcpy(tmp + 12, payload, n);
    storeLe16(tmp + 12 + n, interboardCrc16(tmp, 12 + n));
    std::memcpy(out, tmp, n + 14);
    return n + 14;
}
bool header(const FrameMeta &m, MessageId id, BoardRole role) {
    return m.message_id == id && m.sender_role == role;
}
}
int InterBoardCodec::decodeHeartbeat(const FrameMeta &m, const std::uint8_t *p, std::size_t n, BoardHeartbeat &out) {
    if (!p || n != 24 || m.message_id != MessageId::Heartbeat ||
        (m.sender_role != BoardRole::GimbalController && m.sender_role != BoardRole::ChassisController) ||
        p[0] != static_cast<std::uint8_t>(m.sender_role) || p[1] > 6 || p[2] > 1 || p[3] > 1 || !loadLe64(p + 12))
        return -EINVAL;
    BoardHeartbeat t{};
    t.role = static_cast<BoardRole>(p[0]);
    t.safety_state = static_cast<SafetyState>(p[1]);
    t.ready = p[2];
    t.sync_requested = p[3];
    t.active_reasons = loadLe32(p + 4);
    t.sender_uptime_ms = loadLe32(p + 8);
    t.sender_boot_id = loadLe64(p + 12);
    t.resume_generation = loadLe32(p + 20);
    t.stamp = {m.local_receive_ms, m.frame_sequence, true};
    out = t;
    return 0;
}
int InterBoardCodec::encodeHeartbeat(const BoardHeartbeat &t, std::uint32_t seq, std::uint8_t *out, std::size_t cap) {
    std::uint8_t p[24]{};
    p[0] = static_cast<std::uint8_t>(t.role);
    p[1] = static_cast<std::uint8_t>(t.safety_state);
    p[2] = t.ready;
    p[3] = t.sync_requested;
    storeLe32(p + 4, t.active_reasons);
    storeLe32(p + 8, t.sender_uptime_ms);
    storeLe64(p + 12, t.sender_boot_id);
    storeLe32(p + 20, t.resume_generation);
    BoardHeartbeat check{};
    int ret = decodeHeartbeat({t.role, MessageId::Heartbeat, seq, 0}, p, sizeof(p), check);
    if (ret < 0)
        return ret;
    return frame(t.role, MessageId::Heartbeat, seq, p, sizeof(p), out, cap);
}
int InterBoardCodec::decodeChassisControl(const FrameMeta &m, const std::uint8_t *p, std::size_t n,
                                          RemoteChassisControl &out) {
    if (!p || n != 36 || !header(m, MessageId::ChassisControl, BoardRole::GimbalController) || p[0] > 2 || p[1] > 4 ||
        p[2] > 2 || p[3])
        return -EINVAL;
    RemoteChassisControl t{};
    t.command.mode = static_cast<ChassisMode>(p[0]);
    t.command.source = static_cast<ControlSource>(p[1]);
    t.global_action = static_cast<SafetyAction>(p[2]);
    t.command.vx_m_s = loadFloatLe(p + 4);
    t.command.vy_m_s = loadFloatLe(p + 8);
    t.command.wz_rad_s = loadFloatLe(p + 12);
    if (!std::isfinite(t.command.vx_m_s) || !std::isfinite(t.command.vy_m_s) || !std::isfinite(t.command.wz_rad_s))
        return -EINVAL;
    t.active_reasons = loadLe32(p + 16);
    t.command.stamp = {m.local_receive_ms, loadLe32(p + 20), true};
    t.receiver_boot_id = loadLe64(p + 24);
    t.resume_generation = loadLe32(p + 32);
    t.stamp = t.command.stamp;
    out = t;
    return 0;
}
int InterBoardCodec::encodeChassisControl(const RemoteChassisControl &t, std::uint32_t seq, std::uint8_t *out,
                                          std::size_t cap) {
    std::uint8_t p[36]{};
    p[0] = static_cast<std::uint8_t>(t.command.mode);
    p[1] = static_cast<std::uint8_t>(t.command.source);
    p[2] = static_cast<std::uint8_t>(t.global_action);
    storeFloatLe(p + 4, t.command.vx_m_s);
    storeFloatLe(p + 8, t.command.vy_m_s);
    storeFloatLe(p + 12, t.command.wz_rad_s);
    storeLe32(p + 16, t.active_reasons);
    storeLe32(p + 20, t.command.stamp.sequence);
    storeLe64(p + 24, t.receiver_boot_id);
    storeLe32(p + 32, t.resume_generation);
    RemoteChassisControl check{};
    int ret = decodeChassisControl({BoardRole::GimbalController, MessageId::ChassisControl, seq, 0}, p, sizeof(p),
                                   check);
    if (ret < 0)
        return ret;
    return frame(BoardRole::GimbalController, MessageId::ChassisControl, seq, p, sizeof(p), out, cap);
}
int InterBoardCodec::decodeChassisConstraint(const FrameMeta &m, const std::uint8_t *p, std::size_t n,
                                             ChassisConstraint &out) {
    if (!p || n != 20 || !header(m, MessageId::ChassisConstraint, BoardRole::GimbalController) || p[0] > 1 ||
        p[1] > 1 || p[2] > 1 || p[3])
        return -EINVAL;
    ChassisConstraint t{};
    t.output.valid = p[0];
    t.output.enabled = p[1];
    t.power_valid = p[2];
    t.power_limit_w = loadFloatLe(p + 4);
    t.buffer_energy_j = loadFloatLe(p + 8);
    if (!std::isfinite(t.power_limit_w) || t.power_limit_w < 0 || !std::isfinite(t.buffer_energy_j) ||
        t.buffer_energy_j < 0)
        return -EINVAL;
    t.output_age_ms = loadLe32(p + 12);
    t.power_age_ms = loadLe32(p + 16);
    t.stamp = {m.local_receive_ms, m.frame_sequence, true};
    t.output.stamp = t.stamp;
    t.output.stamp.valid = t.output.valid;
    out = t;
    return 0;
}
int InterBoardCodec::encodeChassisConstraint(const ChassisConstraint &t, std::uint32_t seq, std::uint8_t *out,
                                             std::size_t cap) {
    std::uint8_t p[20]{};
    p[0] = t.output.valid;
    p[1] = t.output.enabled;
    p[2] = t.power_valid;
    storeFloatLe(p + 4, t.power_limit_w);
    storeFloatLe(p + 8, t.buffer_energy_j);
    storeLe32(p + 12, t.output_age_ms);
    storeLe32(p + 16, t.power_age_ms);
    ChassisConstraint check{};
    int ret = decodeChassisConstraint({BoardRole::GimbalController, MessageId::ChassisConstraint, seq, 0}, p, sizeof(p),
                                      check);
    if (ret < 0)
        return ret;
    return frame(BoardRole::GimbalController, MessageId::ChassisConstraint, seq, p, sizeof(p), out, cap);
}
int InterBoardCodec::decodeChassisFeedback(const FrameMeta &m, const std::uint8_t *p, std::size_t n,
                                           ChassisFeedbackSummary &out) {
    if (!p || n != 32 || !header(m, MessageId::ChassisFeedback, BoardRole::ChassisController) || p[0] > 5 || p[1] > 6 ||
        p[2] > 1 || p[3] > 1 || (loadLe32(p + 12) & ~3u))
        return -EINVAL;
    ChassisFeedbackSummary t{};
    t.execution_state = static_cast<ExecutionState>(p[0]);
    t.safety_state = static_cast<SafetyState>(p[1]);
    t.ready = p[2];
    t.armed = p[3];
    t.active_reasons = loadLe32(p + 4);
    t.last_command_sequence = loadLe32(p + 8);
    t.valid_fields = loadLe32(p + 12);
    t.vx_m_s = loadFloatLe(p + 16);
    t.vy_m_s = loadFloatLe(p + 20);
    t.wz_rad_s = loadFloatLe(p + 24);
    t.power_w = loadFloatLe(p + 28);
    if (!std::isfinite(t.vx_m_s) || !std::isfinite(t.vy_m_s) || !std::isfinite(t.wz_rad_s) ||
        !std::isfinite(t.power_w) || t.power_w < 0)
        return -EINVAL;
    t.stamp = {m.local_receive_ms, m.frame_sequence, true};
    out = t;
    return 0;
}
int InterBoardCodec::encodeChassisFeedback(const ChassisFeedbackSummary &t, std::uint32_t seq, std::uint8_t *out,
                                           std::size_t cap) {
    std::uint8_t p[32]{};
    p[0] = static_cast<std::uint8_t>(t.execution_state);
    p[1] = static_cast<std::uint8_t>(t.safety_state);
    p[2] = t.ready;
    p[3] = t.armed;
    storeLe32(p + 4, t.active_reasons);
    storeLe32(p + 8, t.last_command_sequence);
    storeLe32(p + 12, t.valid_fields);
    storeFloatLe(p + 16, t.vx_m_s);
    storeFloatLe(p + 20, t.vy_m_s);
    storeFloatLe(p + 24, t.wz_rad_s);
    storeFloatLe(p + 28, t.power_w);
    ChassisFeedbackSummary check{};
    int ret = decodeChassisFeedback({BoardRole::ChassisController, MessageId::ChassisFeedback, seq, 0}, p, sizeof(p),
                                    check);
    if (ret < 0)
        return ret;
    return frame(BoardRole::ChassisController, MessageId::ChassisFeedback, seq, p, sizeof(p), out, cap);
}
}
