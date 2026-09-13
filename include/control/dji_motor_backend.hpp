#pragma once

#include <control/motor_backend.hpp>
#include <drivers/motor/dji_bus.hpp>

namespace skywalker::control {

// One exclusive single-motor DJI Bus. Multiple axes on one CAN need a shared
// group sender instead: independent group frames would overwrite other slots.
class DjiMotorBackend final : public MotorBackend {
public:
    explicit DjiMotorBackend(const device *motor) : motor_(motor) {
    }
    int describe(MotorInfo &info) override;
    int prepare() override;
    int configure() override;
    int pollPrepare(std::uint64_t now_ms) override;
    int resetMeasurementReference() override;
    int read(MotorMeasurement &measurement) override;
    int arm() override;
    int write(float effort_a) override;
    int flush() override;
    int stop() override;
    const motor::dji::FlushReport &report() const {
        return report_;
    }
    const motor::dji::FlushReport &stopReport() const {
        return stop_report_;
    }

private:
    bool configured_ = false;
    std::uint64_t next_probe_ms_ = 0;
    const device *motor_;
    motor::dji::Bus bus_{};
    motor::dji::Descriptor descriptor_{};
    motor::dji::FlushReport report_{};
    motor::dji::FlushReport stop_report_{};
};

} // namespace skywalker::control
