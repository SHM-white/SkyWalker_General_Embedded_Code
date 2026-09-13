#pragma once
#include <array>
#include <cstddef>
#include <robotics/messages/interboard.hpp>
namespace skywalker::communication {
using robotics::BoardRole;
enum class MessageId : std::uint16_t {
    Heartbeat = 0x0001,
    ChassisControl = 0x0101,
    ChassisConstraint = 0x0102,
    ChassisFeedback = 0x0103,
    ChassisFault = 0x0104,
    GimbalFeedback = 0x0201,
    SystemEvent = 0x0301
};
constexpr std::size_t kMaxPayload = 128, kMaxFrame = 142;
struct FrameMeta {
    BoardRole sender_role = BoardRole::Unknown;
    MessageId message_id = MessageId::Heartbeat;
    std::uint32_t frame_sequence = 0;
    std::uint64_t local_receive_ms = 0;
};
struct InterBoardFrame {
    FrameMeta meta{};
    std::array<std::uint8_t, kMaxPayload> payload{};
    std::size_t size = 0;
};
std::uint16_t interboardCrc16(const std::uint8_t *data, std::size_t size);
int messageIndex(MessageId id);
}
