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
// USB / legacy active serial frames. Firmware 2.x RS485 uses the parser below.
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

class DmImuRs485Sink {
public:
    virtual ~DmImuRs485Sink() = default;
    // Return 1 for a published update, 0 for an ignored/error reply, or a negative error.
    virtual int acceptDmRs485(std::uint8_t rid, std::uint8_t response_code, const Update &,
                             core::TimeUs first_rx_us, core::TimeUs last_rx_us) = 0;
};

// Firmware 2.0.3.0, manual V1.3 appendix D: 24 bytes, no CRC field.
class DmImuRs485Parser {
public:
    struct Config {
        std::uint8_t id = 1;
        core::TimeUs assembly_timeout_us = 20000;
    };
    struct Statistics {
        std::uint32_t frames = 0, invalid_frames = 0, assembly_timeouts = 0;
    };
    static constexpr std::size_t frame_size = 24;
    explicit DmImuRs485Parser(const Config &c) : config_(c) {
    }
    int init();
    int encodeRead(std::uint8_t rid, std::uint8_t *out, std::size_t capacity) const;
    int consume(const std::uint8_t *, std::size_t, core::TimeUs, DmImuRs485Sink &);
    void discardPartial() {
        used_ = 0;
    }
    Statistics statistics() const {
        return stats_;
    } // Owner thread only.

private:
    void drop();
    Config config_;
    std::uint8_t bytes_[frame_size]{};
    core::TimeUs times_[frame_size]{};
    std::size_t used_ = 0;
    Statistics stats_{};
    bool initialized_ = false;
};
}
