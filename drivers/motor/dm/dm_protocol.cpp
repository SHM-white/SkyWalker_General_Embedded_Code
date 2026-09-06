#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <errno.h>

#include <drivers/motor/dm_protocol.hpp>

namespace skywalker::motor::dm {
namespace {

constexpr float kKpMin = 0.0f;
constexpr float kKpMax = 500.0f;
constexpr float kKdMin = 0.0f;
constexpr float kKdMax = 5.0f;

bool validLimits(const Limits &limits)
{
    return std::isfinite(limits.position_max_rad) &&
           limits.position_max_rad > 0.0f &&
           std::isfinite(limits.velocity_max_rad_s) &&
           limits.velocity_max_rad_s > 0.0f &&
           std::isfinite(limits.torque_max_nm) &&
           limits.torque_max_nm > 0.0f;
}

int floatToUint(float value,
                float minimum,
                float maximum,
                unsigned int bits,
                std::uint32_t &out)
{
    if (!std::isfinite(value) || !std::isfinite(minimum) ||
        !std::isfinite(maximum) || minimum >= maximum ||
        (bits != 12u && bits != 16u)) {
        return -EINVAL;
    }
    if (value < minimum || value > maximum) {
        return -ERANGE;
    }

    const std::uint32_t full_scale = (1u << bits) - 1u;
    const float scaled = (value - minimum) *
                         static_cast<float>(full_scale) /
                         (maximum - minimum);
    std::uint32_t encoded = static_cast<std::uint32_t>(scaled);
    if (encoded > full_scale) {
        encoded = full_scale;
    }
    out = encoded;
    return 0;
}

float uintToFloat(std::uint32_t value,
                  float minimum,
                  float maximum,
                  unsigned int bits)
{
    const std::uint32_t full_scale = (1u << bits) - 1u;
    return static_cast<float>(value) * (maximum - minimum) /
           static_cast<float>(full_scale) + minimum;
}

void writeLeFloat(float value, std::uint8_t *out)
{
    static_assert(sizeof(float) == sizeof(std::uint32_t),
                  "Damiao protocol requires 32-bit IEEE-754 float");
    std::uint32_t bits = 0u;
    std::memcpy(&bits, &value, sizeof(bits));
    out[0] = static_cast<std::uint8_t>(bits & 0xFFu);
    out[1] = static_cast<std::uint8_t>((bits >> 8) & 0xFFu);
    out[2] = static_cast<std::uint8_t>((bits >> 16) & 0xFFu);
    out[3] = static_cast<std::uint8_t>((bits >> 24) & 0xFFu);
}

DriveStatus decodeStatus(std::uint8_t raw)
{
    switch (raw) {
    case 0x0u: return DriveStatus::Disabled;
    case 0x1u: return DriveStatus::Enabled;
    case 0x8u: return DriveStatus::OverVoltage;
    case 0x9u: return DriveStatus::UnderVoltage;
    case 0xAu: return DriveStatus::OverCurrent;
    case 0xBu: return DriveStatus::MosOverTemperature;
    case 0xCu: return DriveStatus::MotorOverTemperature;
    case 0xDu: return DriveStatus::CommunicationLost;
    case 0xEu: return DriveStatus::Overload;
    default: return DriveStatus::Unknown;
    }
}

bool validSpecialCommand(SpecialCommand command)
{
    switch (command) {
    case SpecialCommand::ClearError:
    case SpecialCommand::Enable:
    case SpecialCommand::Disable:
    case SpecialCommand::SaveZero:
        return true;
    }
    return false;
}

} // namespace

bool isFaultStatus(DriveStatus status)
{
    return status != DriveStatus::Disabled &&
           status != DriveStatus::Enabled;
}

int controlFrameId(ControlMode mode,
                   std::uint16_t motor_id,
                   std::uint16_t &out)
{
    if (motor_id == 0u || motor_id > 15u) {
        return -ERANGE;
    }

    std::uint16_t offset = 0u;
    switch (mode) {
    case ControlMode::Mit:
        break;
    case ControlMode::PositionVelocity:
        offset = 0x100u;
        break;
    case ControlMode::Velocity:
        offset = 0x200u;
        break;
    default:
        return -EINVAL;
    }

    const std::uint16_t id = static_cast<std::uint16_t>(offset + motor_id);
    if (id > CAN_STD_ID_MASK) {
        return -ERANGE;
    }
    out = id;
    return 0;
}

int buildMitFrame(std::uint16_t motor_id,
                  const Limits &limits,
                  const MitCommand &command,
                  struct can_frame &out)
{
    if (!validLimits(limits)) {
        return -EINVAL;
    }

    std::uint16_t command_id = 0u;
    int ret = controlFrameId(ControlMode::Mit, motor_id, command_id);
    if (ret < 0) {
        return ret;
    }

    std::uint32_t position = 0u;
    std::uint32_t velocity = 0u;
    std::uint32_t kp = 0u;
    std::uint32_t kd = 0u;
    std::uint32_t torque = 0u;

    ret = floatToUint(command.position_rad,
                      -limits.position_max_rad,
                      limits.position_max_rad,
                      16u,
                      position);
    if (ret < 0) return ret;
    ret = floatToUint(command.velocity_rad_s,
                      -limits.velocity_max_rad_s,
                      limits.velocity_max_rad_s,
                      12u,
                      velocity);
    if (ret < 0) return ret;
    ret = floatToUint(command.kp, kKpMin, kKpMax, 12u, kp);
    if (ret < 0) return ret;
    ret = floatToUint(command.kd, kKdMin, kKdMax, 12u, kd);
    if (ret < 0) return ret;
    ret = floatToUint(command.torque_ff_nm,
                      -limits.torque_max_nm,
                      limits.torque_max_nm,
                      12u,
                      torque);
    if (ret < 0) return ret;

    struct can_frame next{};
    next.id = command_id;
    next.flags = 0u;
    next.dlc = 8u;
    next.data[0] = static_cast<std::uint8_t>(position >> 8);
    next.data[1] = static_cast<std::uint8_t>(position);
    next.data[2] = static_cast<std::uint8_t>(velocity >> 4);
    next.data[3] = static_cast<std::uint8_t>(((velocity & 0x0Fu) << 4) |
                                             (kp >> 8));
    next.data[4] = static_cast<std::uint8_t>(kp);
    next.data[5] = static_cast<std::uint8_t>(kd >> 4);
    next.data[6] = static_cast<std::uint8_t>(((kd & 0x0Fu) << 4) |
                                             (torque >> 8));
    next.data[7] = static_cast<std::uint8_t>(torque);
    out = next;
    return 0;
}

int buildPositionVelocityFrame(std::uint16_t motor_id,
                               float position_rad,
                               float velocity_rad_s,
                               struct can_frame &out)
{
    if (!std::isfinite(position_rad) || !std::isfinite(velocity_rad_s)) {
        return -EINVAL;
    }

    std::uint16_t command_id = 0u;
    const int ret = controlFrameId(ControlMode::PositionVelocity,
                                   motor_id,
                                   command_id);
    if (ret < 0) {
        return ret;
    }

    struct can_frame next{};
    next.id = command_id;
    next.flags = 0u;
    next.dlc = 8u;
    writeLeFloat(position_rad, &next.data[0]);
    writeLeFloat(velocity_rad_s, &next.data[4]);
    out = next;
    return 0;
}

int buildVelocityFrame(std::uint16_t motor_id,
                       float velocity_rad_s,
                       struct can_frame &out)
{
    if (!std::isfinite(velocity_rad_s)) {
        return -EINVAL;
    }

    std::uint16_t command_id = 0u;
    const int ret = controlFrameId(ControlMode::Velocity,
                                   motor_id,
                                   command_id);
    if (ret < 0) {
        return ret;
    }

    struct can_frame next{};
    next.id = command_id;
    next.flags = 0u;
    next.dlc = 4u;
    writeLeFloat(velocity_rad_s, &next.data[0]);
    out = next;
    return 0;
}

int buildSpecialFrame(ControlMode mode,
                      std::uint16_t motor_id,
                      SpecialCommand command,
                      struct can_frame &out)
{
    if (!validSpecialCommand(command)) {
        return -EINVAL;
    }

    std::uint16_t command_id = 0u;
    const int ret = controlFrameId(mode, motor_id, command_id);
    if (ret < 0) {
        return ret;
    }

    struct can_frame next{};
    next.id = command_id;
    next.flags = 0u;
    next.dlc = 8u;
    for (std::size_t i = 0; i < 7u; ++i) {
        next.data[i] = 0xFFu;
    }
    next.data[7] = static_cast<std::uint8_t>(command);
    out = next;
    return 0;
}

int decodeFeedback(const struct can_frame &frame,
                   std::uint8_t expected_motor_id,
                   const Limits &limits,
                   DecodedFeedback &out)
{
    if (!validLimits(limits)) {
        return -EINVAL;
    }
    if (expected_motor_id == 0u || expected_motor_id > 15u) {
        return -ERANGE;
    }
    if (frame.dlc != 8u ||
        (frame.flags & (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF)) != 0u) {
        return -EBADMSG;
    }

    const std::uint8_t motor_id = frame.data[0] & 0x0Fu;
    if (motor_id != expected_motor_id) {
        return -ENOENT;
    }

    DecodedFeedback next{};
    next.raw.motor_id = motor_id;
    next.raw.status = decodeStatus(frame.data[0] >> 4);
    next.raw.position_raw =
        static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(frame.data[1]) << 8) |
            frame.data[2]);
    next.raw.velocity_raw =
        static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(frame.data[3]) << 4) |
            (frame.data[4] >> 4));
    next.raw.torque_raw =
        static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(frame.data[4] & 0x0Fu) << 8) |
            frame.data[5]);
    next.raw.mos_temperature_c = frame.data[6];
    next.raw.rotor_temperature_c = frame.data[7];

    next.position_rad = uintToFloat(next.raw.position_raw,
                                    -limits.position_max_rad,
                                    limits.position_max_rad,
                                    16u);
    next.velocity_rad_s = uintToFloat(next.raw.velocity_raw,
                                      -limits.velocity_max_rad_s,
                                      limits.velocity_max_rad_s,
                                      12u);
    next.torque_nm = uintToFloat(next.raw.torque_raw,
                                 -limits.torque_max_nm,
                                 limits.torque_max_nm,
                                 12u);
    out = next;
    return 0;
}

} // namespace skywalker::motor::dm
