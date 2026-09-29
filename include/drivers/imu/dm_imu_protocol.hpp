#pragma once
#include <cstddef>
#include <drivers/imu/imu_state.hpp>
namespace skywalker::imu {
std::uint16_t dmImuCrc16(const std::uint8_t *, std::size_t);
class DmImuSink {
public:
    virtual ~DmImuSink() = default;
    virtual int acceptDm(const Update &) = 0;
};
// Active RS485/USB frames only. No configuration writes or CAN implementation.
class DmImuParser {
public:
    struct Config {
        std::uint8_t id = 1;
        core::TimeUs assembly_timeout_us = 20000;
    };
    struct Statistics {
        std::uint32_t frames = 0, crc_errors = 0, invalid_frames = 0, assembly_timeouts = 0;
    };
    explicit DmImuParser(const Config &c) : config_(c) {
    }
    int init();
    void discardPartial() {
        used_ = 0;
    }
    int consume(const std::uint8_t *, std::size_t, core::TimeUs, DmImuSink &);
    Statistics statistics() const {
        return stats_;
    } // Owner thread only.
private:
    void drop();
    Config config_;
    std::uint8_t bytes_[23]{};
    core::TimeUs times_[23]{};
    std::size_t used_ = 0;
    Statistics stats_{};
    bool initialized_ = false;
};
}
