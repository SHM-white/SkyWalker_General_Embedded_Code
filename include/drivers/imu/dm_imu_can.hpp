#pragma once
#include <drivers/imu/imu.hpp>
#include <zephyr/device.h>
namespace skywalker::imu {
// Reserved abstract interface only. No CAN receiver/decoder or Kconfig is supplied.
// Future implementation must resolve the firmware-specific configuration protocol.
class DmImuCanSource : public ImuSource {
public:
    struct Config {
        const device *can = nullptr;
        std::uint16_t feedback_id = 0;
        core::OrientationReference reference{2, 1};
        core::Quaternion sensor_to_body{};
        Freshness freshness{};
    };
    // init/service/snapshot remain pure virtual through ImuSource.
};
}
