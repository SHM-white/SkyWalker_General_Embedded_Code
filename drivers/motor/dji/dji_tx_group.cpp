#include <drivers/motor/dji_tx_group.hpp>
#include <drivers/motor/dji_protocol.hpp>
#include <drivers/motor/dji_m3508.hpp>
#include <zephyr/drivers/can.h>

int skywalker::motor::dji::TxGroup::init(const device *can, std::uint16_t command_id)
{
    if (can == nullptr) return -EINVAL;
    if (!device_is_ready(can)) return -ENODEV;
    can_ = can;
    command_id_ = command_id;
    for (auto &motor : motors_) {
        motor = nullptr;
    }
    return 0;
}

int skywalker::motor::dji::TxGroup::bindMotor(const device *motor)
{
    if (motor == nullptr) return -EINVAL;
    std::uint16_t command_id = 0;
    std::uint8_t command_slot = 0;

    int ret = skywalker::motor::dji::m3508::getCommandInfo(motor, command_id, command_slot);
    if (ret < 0) return ret;

    if (command_id != command_id_) return -EINVAL;
    if (command_slot >= 4) return -EINVAL;

    motors_[command_slot] = motor;
    return 0;
}

int skywalker::motor::dji::TxGroup::send()
{
    if (can_ == nullptr) return -ENODEV;

    std::int16_t command[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i)
    {
        if (motors_[i] == nullptr) continue;
        int ret = skywalker::motor::dji::m3508::getCurrentRaw(motors_[i], command[i]);
        if (ret < 0) return ret;
    }
    can_frame frame{};
    buildGroupCurrentFrame(frame, command_id_, command);
    return can_send(can_, &frame, K_MSEC(1), nullptr, nullptr);
}

void skywalker::motor::dji::TxGroup::clear()
{
    for (auto *motor : motors_)
    {
        if (motor == nullptr) continue;
        skywalker::motor::dji::m3508::setCurrentRaw(motor, 0);
    }
}
