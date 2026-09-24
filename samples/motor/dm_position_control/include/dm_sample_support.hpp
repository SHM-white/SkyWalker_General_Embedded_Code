#pragma once

#include <zephyr/device.h>

#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/dm_motor.hpp>
#include <drivers/motor/motor.hpp>

namespace skywalker::samples::dm {

struct Session {
    Session(const device *can_device, motor::dm::Config motor_config)
        : can(can_device), config(motor_config), motor(config), bus(can) {
    }

    const device *can;
    motor::dm::Config config;
    motor::Motor motor;
    motor::CanBus bus;
    motor::dm::Descriptor descriptor{};
};

// CAN routes are installed before XT30_1 is enabled on MC02.
int prepare(Session &session);
int arm(Session &session);
int readSafeFeedback(const Session &session, float velocity_abs_max_rad_s, float temperature_max_c,
                     motor::MotorSnapshot &snapshot);
int flush(Session &session);
int stop(Session &session);
int stopAfterFailure(Session &session, int original_error);

} // namespace skywalker::samples::dm
