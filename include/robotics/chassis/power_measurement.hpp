#pragma once

#include <core/measurement.hpp>

namespace skywalker::robotics {

enum class PowerMeasurementSource : unsigned char { None, Sensor, Referee, Estimated };

// A measurement producer owns this stamp. Reading or transmitting the sample
// must preserve it; optional voltage/current alone do not make power valid.
struct PowerMeasurement {
    float power_w = 0, bus_voltage_v = 0, bus_current_a = 0;
    bool valid = false, voltage_current_valid = false;
    PowerMeasurementSource source = PowerMeasurementSource::None;
    core::Stamp stamp{};
};

class IPowerMeasurementSource {
public:
    virtual ~IPowerMeasurementSource() = default;
    virtual int sample(core::TimeUs now_us, PowerMeasurement &out) = 0;
};

} // namespace skywalker::robotics
