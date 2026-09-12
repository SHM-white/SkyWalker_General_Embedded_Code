#pragma once

#include <control/motor_backend.hpp>
#include <drivers/motor/dji_bus.hpp>

namespace skywalker::control {

// One exclusive single-motor DJI Bus. Multiple axes on one CAN need a shared
// group sender instead: independent group frames would overwrite other slots.
class DjiMotorBackend final : public MotorBackend {
public:
    explicit DjiMotorBackend(const device *motor) : motor_(motor) {}
    int describe(MotorInfo &info) override;
    int prepare() override;
    int read(MotorMeasurement &measurement) override;
    int arm() override;
    int write(float effort_a) override;
    int flush() override;
    int stop() override;
    const motor::dji::FlushReport &report() const { return report_; }
    const motor::dji::FlushReport &stopReport() const { return stop_report_; }

private:
    const device *motor_;
    motor::dji::Bus bus_{};
    motor::dji::Descriptor descriptor_{};
    motor::dji::FlushReport report_{};
    motor::dji::FlushReport stop_report_{};
};

} // namespace skywalker::control
