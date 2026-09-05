#pragma once

#include <cstdint>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/spinlock.h>

#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>

namespace skywalker::motor::dji::internal {

struct Profile {
    Model model;
    std::uint8_t max_motor_id;
    std::uint16_t feedback_base;
    std::uint16_t low_command_id;
    std::uint16_t high_command_id;
    std::int16_t command_raw_max;
    float protocol_current_max_a;
    bool temperature_valid;
};

struct Endpoint {
    std::uint16_t feedback_id = 0;
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;
};

struct CommandSnapshot {
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;
    std::int16_t command_raw = 0;
};

struct DjiConfig {
    const struct device *can;
    const Profile *profile;
    std::uint32_t motor_id;
    std::uint32_t current_limit_ma;
    std::uint32_t gear_ratio_num;
    std::uint32_t gear_ratio_den;
};

struct DjiData {
    Feedback feedback{};
    RawFeedback raw_feedback{};
    Endpoint endpoint{};
    float current_limit_a = 0.0f;
    float gear_ratio = 0.0f;

    std::int16_t command_raw = 0;
    std::uint64_t command_stamp_ms = 0;
    std::uint64_t command_generation = 0;
    std::uint64_t command_epoch = 0;
    std::uint64_t active_epoch = 0;
    bool armed = false;
    bool fault_latched = false;

    std::uint16_t last_encoder = 0;
    std::int64_t total_encoder_ticks = 0;
    bool has_encoder = false;
    std::uint64_t last_rx_ms = 0;

    struct k_spinlock lock{};
    int rx_filter_id = -1;
};

extern const Profile kM3508C620Profile;
extern const Profile kM2006C610Profile;
extern const Profile kGM6020CurrentProfile;
extern const skywalker::motor::Api dji_motor_api;

int resolveEndpoint(const Profile &profile, std::uint8_t motor_id, Endpoint &out);
int currentToRaw(const Profile &profile, float current_a, std::int16_t &out);
int rawToCurrent(const Profile &profile, std::int16_t raw, float &out);

int djiMotorInit(const struct device *dev);
bool isDjiMotor(const struct device *dev);
bool feedbackReady(const struct device *dev);
int armMotor(const struct device *dev, std::uint64_t epoch, std::uint64_t now_ms);
void prepareMotorStop(const struct device *dev, bool latch_fault);
void clearMotorFault(const struct device *dev);
int snapshotCommand(const struct device *dev, std::uint64_t expected_epoch, std::uint64_t now_ms, CommandSnapshot &out);

} // namespace skywalker::motor::dji::internal

#define DJI_MOTOR_DEFINE(inst, profile_symbol)                              \
    static skywalker::motor::dji::internal::DjiData                        \
        dji_data_##inst;                                                    \
    static const skywalker::motor::dji::internal::DjiConfig                \
        dji_config_##inst = {                                               \
            DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                  \
            &(profile_symbol),                                              \
            DT_INST_PROP(inst, motor_id),                                   \
            DT_INST_PROP(inst, current_limit_ma),                           \
            DT_INST_PROP(inst, gear_ratio_num),                             \
            DT_INST_PROP(inst, gear_ratio_den),                             \
        };                                                                  \
    DEVICE_DT_INST_DEFINE(                                                  \
        inst,                                                               \
        skywalker::motor::dji::internal::djiMotorInit,                      \
        nullptr,                                                            \
        &dji_data_##inst,                                                   \
        &dji_config_##inst,                                                 \
        POST_KERNEL,                                                        \
        CONFIG_SKYWALKER_MOTOR_INIT_PRIORITY,                               \
        &skywalker::motor::dji::internal::dji_motor_api);
