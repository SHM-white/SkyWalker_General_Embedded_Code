#pragma once

#include <cstdint>

#include <zephyr/drivers/can.h>

namespace skywalker::motor::dm {

enum class ControlMode : std::uint8_t {
    Mit = 0,
    PositionVelocity,
    Velocity,
};

enum class DriveStatus : std::uint8_t {
    Disabled = 0x0,
    Enabled = 0x1,
    OverVoltage = 0x8,
    UnderVoltage = 0x9,
    OverCurrent = 0xA,
    MosOverTemperature = 0xB,
    MotorOverTemperature = 0xC,
    CommunicationLost = 0xD,
    Overload = 0xE,
    Unknown = 0xF,
};

enum class SpecialCommand : std::uint8_t {
    ClearError = 0xFB,
    Enable = 0xFC,
    Disable = 0xFD,
    SaveZero = 0xFE,
};

struct Limits {
    float position_max_rad = 0.0f;
    float velocity_max_rad_s = 0.0f;
    float torque_max_nm = 0.0f;
};

struct MitCommand {
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque_ff_nm = 0.0f;
};

struct RawFeedback {
    std::uint8_t motor_id = 0;
    DriveStatus status = DriveStatus::Unknown;
    std::uint16_t position_raw = 0;
    std::uint16_t velocity_raw = 0;
    std::uint16_t torque_raw = 0;
    std::uint8_t mos_temperature_c = 0;
    std::uint8_t rotor_temperature_c = 0;
    std::uint64_t timestamp_ms = 0;
};

struct DecodedFeedback {
    RawFeedback raw{};
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float torque_nm = 0.0f;
};

bool isFaultStatus(DriveStatus status);

int controlFrameId(ControlMode mode, std::uint16_t motor_id, std::uint16_t &out);

int buildMitFrame(std::uint16_t motor_id, const Limits &limits, const MitCommand &command, struct can_frame &out);

int buildPositionVelocityFrame(std::uint16_t motor_id, float position_rad, float velocity_rad_s, struct can_frame &out);

int buildVelocityFrame(std::uint16_t motor_id, float velocity_rad_s, struct can_frame &out);

int buildSpecialFrame(ControlMode mode, std::uint16_t motor_id, SpecialCommand command, struct can_frame &out);

int decodeFeedback(const struct can_frame &frame, std::uint8_t expected_motor_id, const Limits &limits,
                   DecodedFeedback &out);

} // namespace skywalker::motor::dm
