#pragma once

#include <cstddef>
#include <cstdint>
#include <zephyr/device.h>

#include <drivers/motor/dji_motor.hpp>

namespace skywalker::motor::dji {

enum class BusState : std::uint8_t {
    Uninitialized = 0,
    Safe,
    Armed,
    Fault,
};

struct FlushReport {
    int preparation_error = 0;
    int command_tx_error = 0;
    int zero_tx_error = 0;
    std::uint16_t target_failed_command_id = 0;
    std::uint16_t zero_failed_command_id = 0;
    std::uint8_t preparation_failed_slot = 0xFFu;
    std::uint8_t zero_groups_expected = 0;
    std::uint8_t zero_groups_attempted = 0;
    std::uint8_t zero_groups_succeeded = 0;
    bool zero_sent = false;
};

class Bus {
public:
    Bus() = default;
    Bus(const Bus &) = delete;
    Bus &operator=(const Bus &) = delete;
    Bus(Bus &&) = delete;
    Bus &operator=(Bus &&) = delete;

    int init(const struct device *can);
    int attach(const struct device *motor);
    int arm(FlushReport &report);
    int flush(FlushReport &report);
    int stop(FlushReport &report);
    int recover(FlushReport &report);
    BusState state() const;

private:
    int groupIndex(std::uint16_t command_id) const;
    int sendZeros(FlushReport &report);
    int enterFaultAndZero(FlushReport &report);

    const struct device *can_ = nullptr;
    const struct device *
        motors_[CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS]{};
    Descriptor
        descriptors_[CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS]{};
    std::uint16_t
        group_ids_[CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS]{};
    std::size_t motor_count_ = 0;
    std::size_t group_count_ = 0;
    std::uint64_t lifecycle_epoch_ = 0;
    BusState state_ = BusState::Uninitialized;
};

} // namespace skywalker::motor::dji
