#pragma once
#include <zephyr/device.h>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/dm_motor.hpp>
namespace skywalker::samples::dm {
struct Session {
    Session(const device *dev, motor::dm::Config cfg) : can(dev), config(cfg), motor(cfg), bus(dev) {
    }
    const device *can;
    motor::dm::Config config;
    motor::Motor motor;
    motor::CanBus bus;
    motor::dm::Descriptor descriptor{};
};
int prepare(Session &session);
int arm(Session &session);
int flush(Session &session);
int stop(Session &session);
}
