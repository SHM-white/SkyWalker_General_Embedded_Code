#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/calibration.hpp>
#include <core/clock.hpp>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
using namespace skywalker;
using namespace skywalker::robotics;
namespace installed = skywalker::robotics::vehicle;
using installed::DiagnosticReason;

int main() {
    installed::Diagnostics diagnostics;
    diagnostics.start("gimbal_io", "gimbal", "no actuators");
    for (;;) {
        diagnostics.status({.reason = DiagnosticReason::Ready, .valid = true});
        k_sleep(K_MSEC(20));
    }
}
