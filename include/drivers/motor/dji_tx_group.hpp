#pragma once

#include <cstdint>
#include <zephyr/device.h>

namespace skywalker::motor::dji
{
    
class TxGroup
{
private:
    const struct device *can_ = nullptr;
    std::uint16_t command_id_ = 0;

    const struct device *motors_[4] = {nullptr, nullptr, nullptr, nullptr};
public:
    int init(const struct device *can, std::uint16_t command_id);
    int bindMotor(const struct device *motor);
    int send();
    void clear();
};

} // namespace skywalker::motor::dji
