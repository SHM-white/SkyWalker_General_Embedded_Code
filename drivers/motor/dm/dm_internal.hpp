#pragma once

#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/spinlock.h>

#include <drivers/motor/dm_motor.hpp>
#include <drivers/motor/motor.hpp>

namespace skywalker::motor::dm {

class Bus;

namespace internal {

struct DmConfig {
    const struct device *can;
    Model model;
    std::uint32_t motor_id;
    std::uint32_t master_id;
    ControlMode mode;
    std::uint32_t p_max_millirad;
    std::uint32_t v_max_millirad_s;
    std::uint32_t t_max_millinewton_meter;
    std::uint32_t torque_limit_millinewton_meter;
};

struct DmData {
    Feedback feedback{};
    RawFeedback raw_feedback{};
    DriveStatus drive_status = DriveStatus::Unknown;
    Limits limits{};
    float torque_limit_nm = 0.0f;
    struct can_frame command_frame{};
    std::uint64_t last_rx_ms = 0;
    std::uint64_t command_stamp_ms = 0;
    std::uint64_t command_epoch = 0;
    std::uint64_t active_epoch = 0;
    std::uint64_t armed_at_ms = 0;
    bool command_valid = false;
    bool armed = false;
    bool fault_latched = false;
    Bus *owner = nullptr;
    struct k_spinlock lock{};
};

struct CommandSnapshot {
    struct can_frame frame{};
};

extern const skywalker::motor::Api dm_motor_api;

int dmMotorInit(const struct device *dev);
bool isDmMotor(const struct device *dev);
int claimMotor(const struct device *dev, Bus *owner);
void releaseMotor(const struct device *dev, Bus *owner);
int acceptFeedback(const struct device *dev, const struct can_frame &frame, std::uint64_t now_ms);
int preflightArm(const struct device *dev);
int buildNeutralFrame(const struct device *dev, struct can_frame &out);
int armMotor(const struct device *dev, std::uint64_t epoch, std::uint64_t now_ms, const struct can_frame &neutral_frame);
void prepareMotorStop(const struct device *dev, bool latch_fault);
void clearMotorFault(const struct device *dev);
bool recoveryReady(const struct device *dev, std::uint64_t recovery_started_ms);
int snapshotCommand(const struct device *dev, std::uint64_t expected_epoch, std::uint64_t now_ms, CommandSnapshot &out);

} // namespace internal
} // namespace skywalker::motor::dm

#define DM_MOTOR_DEFINE(inst, model_value)                                                                                                                                                             \
    static skywalker::motor::dm::internal::DmData dm_data_##inst;                                                                                                                                      \
    static const skywalker::motor::dm::internal::DmConfig dm_config_##inst = {                                                                                                                         \
        DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                                                                                                                                                 \
        (model_value),                                                                                                                                                                                 \
        DT_INST_PROP(inst, motor_id),                                                                                                                                                                  \
        DT_INST_PROP(inst, master_id),                                                                                                                                                                 \
        static_cast<skywalker::motor::dm::ControlMode>(DT_INST_ENUM_IDX(inst, control_mode)),                                                                                                          \
        DT_INST_PROP(inst, p_max_millirad),                                                                                                                                                            \
        DT_INST_PROP(inst, v_max_millirad_s),                                                                                                                                                          \
        DT_INST_PROP(inst, t_max_millinewton_meter),                                                                                                                                                   \
        DT_INST_PROP(inst, torque_limit_millinewton_meter),                                                                                                                                            \
    };                                                                                                                                                                                                 \
    DEVICE_DT_INST_DEFINE(inst, skywalker::motor::dm::internal::dmMotorInit, nullptr, &dm_data_##inst, &dm_config_##inst, POST_KERNEL, CONFIG_SKYWALKER_MOTOR_INIT_PRIORITY,                           \
                          &skywalker::motor::dm::internal::dm_motor_api);
