#pragma once
#include <drivers/imu/imu.hpp>
#include <drivers/imu/dm_imu_protocol.hpp>
#include <communication/async_uart.hpp>
namespace skywalker::imu {
// Firmware 2.0.3.0 RS485 reads. Configure/save the module using the manufacturer's tool.
// One owner thread calls init/service; this object and DMA storage must outlive callbacks.
// The transceiver must suppress local TX echo: requests and replies share their layout.
class DmImuRs485Source final : public ImuSource, private DmImuRs485Sink {
public:
    struct Config {
        DmImuRs485Parser::Config protocol{};
        core::OrientationReference reference{2, 1};
        core::Quaternion sensor_to_body{};
        bool device_quaternion_is_world_to_sensor = false;
        float acceleration_scale = 1, angular_velocity_scale = 1;
        Freshness freshness{};
        core::TimeUs response_timeout_us = 50000;
        std::uint8_t max_retries = 2;
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
    int acceptDm(const Update &);
    int acceptDmRs485(std::uint8_t rid, std::uint8_t response_code, const Update &,
                     core::TimeUs first_rx_us, core::TimeUs last_rx_us) override;
    void finishRequest(bool retry);
    void logStatistics(core::TimeUs now);
    enum class Phase { Idle, Sending, AwaitReply };
    Config config_;
    communication::AsyncUart uart_;
    DmImuRs485Parser parser_;
    ImuState state_;
    core::TimeUs retry_us_ = 0;
    std::uint32_t parser_errors_ = 0;
    Phase phase_ = Phase::Idle;
    core::TimeUs request_us_ = 0, response_deadline_us_ = 0, next_request_us_ = 0, next_log_us_ = 0;
    std::uint8_t rid_ = 3, retries_ = 0, last_response_code_ = 0;
    std::uint32_t rx_bytes_ = 0, requests_ = 0, replies_ = 0, timeouts_ = 0;
    int response_error_ = 0;
    bool initialized_ = false;
};
}
