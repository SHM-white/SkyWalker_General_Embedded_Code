#pragma once

#include <control/motor_backend.hpp>
#include <drivers/motor/dm_bus.hpp>

namespace skywalker::control {

class DmMotorBackend final : public MotorBackend {
public:
    // Optional board power hook, called after CAN start/filter attachment.
    // The hook owns any board-specific power-on wait. Only MIT is supported.
    explicit DmMotorBackend(const device *motor, int (*power_on)() = nullptr)
        : motor_(motor), power_on_(power_on) {}
    int describe(MotorInfo &info) override;
    int prepare() override;
    int read(MotorMeasurement &measurement) override;
    int arm() override;
    int write(float effort_nm) override;
    int flush() override;
    int stop() override;
    const motor::dm::TxReport &report() const { return report_; }
    const motor::dm::TxReport &stopReport() const { return stop_report_; }

private:
    const device *motor_;
    int (*power_on_)();
    motor::dm::Bus bus_{};
    motor::dm::Descriptor descriptor_{};
    motor::dm::TxReport report_{};
    motor::dm::TxReport stop_report_{};
    bool position_initialized_ = false;
    float previous_position_rad_ = 0.0f;
    double continuous_position_rad_ = 0.0;
};

} // namespace skywalker::control
