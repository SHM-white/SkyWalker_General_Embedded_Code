#pragma once
#include <drivers/imu/imu.hpp>
#include <drivers/imu/dm_imu_protocol.hpp>
#include <communication/async_uart.hpp>
namespace skywalker::imu {
// Active output only. Configure/save the module using the manufacturer's tool.
// One owner thread calls init/service; this object and DMA storage must outlive callbacks.
class DmImuRs485Source final : public ImuSource, private DmImuSink {
public:
    struct Config {
        DmImuParser::Config protocol{};
        core::OrientationReference reference{2, 1};
        core::Quaternion sensor_to_body{};
        bool device_quaternion_is_world_to_sensor = false;
        float acceleration_scale = 1, angular_velocity_scale = 1;
        Freshness freshness{};
    };
    DmImuRs485Source(const device *uart, communication::AsyncUart::DmaBuffers &dma, const Config &c)
        : config_(c), uart_(uart, dma), parser_(c.protocol), state_(c.freshness) {
    }
    int init() override;
    int service() override;
    Snapshot snapshot() const override {
        return state_.snapshot();
    }
    // Owner only: call after a known remote reset/zero/calibration. Does not send commands.
    int resetReference();

private:
    int acceptDm(const Update &) override;
    Config config_;
    communication::AsyncUart uart_;
    DmImuParser parser_;
    ImuState state_;
    core::TimeUs retry_us_ = 0;
    std::uint32_t parser_errors_ = 0;
    bool initialized_ = false;
};
}
