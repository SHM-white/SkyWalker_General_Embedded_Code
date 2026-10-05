#include <cerrno>
#include <cstdint>
#include <cmath>

#include <zephyr/device.h>
#include "../../../robotics/common/rc_controls.hpp"
#include "../../../robotics/common/sample_diagnostics.hpp"
#if defined(CONFIG_BOARD_DM_MC02) && !defined(MIXED_TOPOLOGY_DJI_SHARED_FRAME)
#include <zephyr/drivers/regulator.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/dm_motor.hpp>
#include <drivers/motor/group.hpp>

LOG_MODULE_REGISTER(mixed_topology, LOG_LEVEL_INF);

namespace {
using namespace skywalker;

// The values below are bench examples. Check every ID, mode, range, gear ratio,
// current/torque limit and direction against the connected hardware.
constexpr motor::Timing kDjiTiming{.feedback_timeout_ms = 20, .command_timeout_ms = 20, .enable_timeout_ms = 100, .retry_interval_ms = 100};
constexpr motor::Timing kDmTiming{.feedback_timeout_ms = 50, .command_timeout_ms = 20, .enable_timeout_ms = 3000, .retry_interval_ms = 100};
constexpr float kDjiTestCurrentA = 0.05f;
constexpr float kDmTestTorqueNm = 0.02f;
constexpr std::int64_t kControlPeriodMs = 5;

motor::dji::Config m3508Id1() {
    return motor::dji::m3508({.id = 1, .current_limit_a = 0.3f, .gear_ratio = 3591.0f / 187.0f, .timing = kDjiTiming});
}

motor::dm::Config dmMit(std::uint8_t id) {
    return motor::dm::j4310Mit({.id = id,
                                .master_id = 0x11,
                                .position_max_rad = 12.5f,
                                .velocity_max_rad_s = 30.0f,
                                .torque_max_nm = 10.0f,
                                .torque_limit_nm = 0.1f,
                                .timing = kDmTiming});
}

#if defined(MIXED_TOPOLOGY_DJI_SHARED_FRAME)
constexpr const char *kTopologyName = "DJI 0x200: M3508 ID1 / M2006 ID2, independent Groups";
#elif defined(MIXED_TOPOLOGY_DM_SHARED_MASTER)
constexpr const char *kTopologyName = "DM MIT ID1 / ID2: Master 0x11, independent Groups";
#elif defined(MIXED_TOPOLOGY_CROSS_CAN_GROUP)
constexpr const char *kTopologyName = "M3508 CAN1 + DM MIT CAN2, one batch Group";
#elif defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
constexpr const char *kTopologyName = "batched M3508/DM across CAN1/CAN2 plus one independent Group per CAN";
#else
#error "Select a MIXED_TOPOLOGY in CMakeLists.txt"
#endif

using samples::control::RcControlAdapter;
using samples::control::DiagnosticScenario;
static_assert(samples::control::diagnostic_scenario == DiagnosticScenario::None ||
              samples::control::diagnostic_scenario == DiagnosticScenario::InputPause ||
              samples::control::diagnostic_scenario == DiagnosticScenario::ExecutionPause);

// Input hysteresis selects a group; endpoint health never changes this request.
struct GroupRequest {
    bool requested = false;
    void update(motor::Group &group, bool allowed, float channel) {
        if (!allowed || channel <= .3f) requested = false;
        else if (channel > .6f) requested = true;
        if (requested) (void)group.enable();
        else group.disable();
    }
};
struct Topology {
#if defined(MIXED_TOPOLOGY_DJI_SHARED_FRAME)
    motor::Motor first{m3508Id1()};
    motor::Motor second{
        motor::dji::m2006({.id = 2, .current_limit_a = 0.3f, .gear_ratio = 36.0f, .timing = kDjiTiming})};
    motor::Group first_group{first};
    motor::Group second_group{second};
    motor::CanBus can1{DEVICE_DT_GET(DT_NODELABEL(can1))};
#elif defined(MIXED_TOPOLOGY_DM_SHARED_MASTER)
    motor::Motor first{dmMit(1)};
    motor::Motor second{dmMit(2)};
    motor::Group first_group{first};
    motor::Group second_group{second};
    motor::CanBus can1{DEVICE_DT_GET(DT_NODELABEL(can1))};
#else
    motor::Motor first{m3508Id1()};
    motor::Motor second{dmMit(1)};
    motor::Group linked_group{first, second};
#if defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
    motor::Motor third{
        motor::dji::m2006({.id = 2, .current_limit_a = 0.3f, .gear_ratio = 36.0f, .timing = kDjiTiming})};
    motor::Motor fourth{dmMit(2)};
    motor::Group third_group{third};
    motor::Group fourth_group{fourth};
#endif
    motor::CanBus can1{DEVICE_DT_GET(DT_NODELABEL(can1))};
    motor::CanBus can2{DEVICE_DT_GET(DT_NODELABEL(can2))};
#endif

    int start() {
#if defined(MIXED_TOPOLOGY_CROSS_CAN_GROUP) || defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        // Attach every linked member before starting either bus.
#if defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        int ret = can1.attach(first, third);
        if (ret == 0)
            ret = can2.attach(second, fourth);
#else
        int ret = can1.attach(first);
        if (ret == 0)
            ret = can2.attach(second);
#endif
        if (ret == 0)
            ret = can1.start();
        if (ret == 0)
            ret = can2.start();
#else
        int ret = can1.attach(first, second);
        if (ret == 0)
            ret = can1.start();
#endif
        if (ret < 0)
            return ret;
#if defined(CONFIG_BOARD_DM_MC02) && !defined(MIXED_TOPOLOGY_DJI_SHARED_FRAME)
        // The DM board power output is controlled here, after CAN routes exist.
        const device *power = DEVICE_DT_GET(DT_NODELABEL(power1));
        if (!device_is_ready(power))
            return -ENODEV;
        ret = regulator_enable(power);
        if (ret < 0)
            return ret;
        k_sleep(K_MSEC(1500));
#endif
        return 0;
    }

    GroupRequest request1{}, request2{}, request3{};
    int last_call_error = 0;
    void record(int error) { if (error < 0) last_call_error = error; }
    void updateRemote(const samples::control::RcControlState &rc, bool source_fresh) {
        const bool allowed = rc.run_allowed && source_fresh;
        const float a = RcControlAdapter::normalize(rc.remote.analog.left_y);
        const float b = RcControlAdapter::normalize(rc.remote.analog.right_y);
        const float c = RcControlAdapter::normalize(rc.remote.analog.wheel);
#if defined(MIXED_TOPOLOGY_CROSS_CAN_GROUP) || defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        request1.update(linked_group, allowed, a);
#if defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        request2.update(third_group, allowed, b);
        request3.update(fourth_group, allowed, c);
#else
        (void)b; (void)c;
#endif
#else
        request1.update(first_group, allowed, a);
        request2.update(second_group, allowed, b);
        (void)c;
#endif
    }
    bool requested() const { return request1.requested || request2.requested || request3.requested; }
    void tick() {
#if defined(MIXED_TOPOLOGY_CROSS_CAN_GROUP) || defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        if (request1.requested) {
            record(first.setCurrent(kDjiTestCurrentA));
            record(second.setTorque(kDmTestTorqueNm));
        }
#if defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        if (request2.requested) record(third.setCurrent(kDjiTestCurrentA));
        if (request3.requested) record(fourth.setTorque(kDmTestTorqueNm));
#endif
        record(can1.commit().error);
        record(can2.commit().error);
#else
#if defined(MIXED_TOPOLOGY_DJI_SHARED_FRAME)
        if (request1.requested) record(first.setCurrent(kDjiTestCurrentA));
        if (request2.requested) record(second.setCurrent(kDjiTestCurrentA));
#else
        if (request1.requested) record(first.setTorque(kDmTestTorqueNm));
        if (request2.requested) record(second.setTorque(kDmTestTorqueNm));
#endif
        record(can1.commit().error);
#endif
    }
    static void logMotor(const char *name, const motor::Motor &drive) {
        const auto v = drive.snapshot();
        LOG_INF("%s requested=%d state=%u fresh=%d fault=%u/%d", name, v.enabled_requested,
            unsigned(v.state), v.feedback_fresh, unsigned(v.last_fault.reason), v.last_fault.error);
    }
    static void logGroup(const char *name, const motor::Group &group) {
        const auto v = group.status();
        LOG_INF("%s members=%u enabled=%u active=%u offline=%u fault=%u", name, unsigned(v.member_count),
            unsigned(v.enabled_count), unsigned(v.active_count), unsigned(v.offline_count), unsigned(v.fault_count));
    }
    void logStatus() const {
        LOG_INF("topology=%s requested=%d call=%d", kTopologyName, requested(), last_call_error);
        logMotor("first", first); logMotor("second", second);
#if defined(MIXED_TOPOLOGY_CROSS_CAN_GROUP) || defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        logGroup("batch", linked_group);
#if defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        logMotor("third", third); logMotor("fourth", fourth);
        logGroup("third", third_group); logGroup("fourth", fourth_group);
#endif
#else
        logGroup("first", first_group); logGroup("second", second_group);
#endif
    }
};
}
int main() {
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                                 samples::control::receiverConfig());
    const int remote_error = remote.start();
    if (remote_error < 0) return remote_error;
    static Topology topology{};
    const int ret = topology.start();
    if (ret < 0) return ret;
    communication::RemoteReceiver::Snapshot snapshot{};
    RcControlAdapter adapter;
    samples::control::SampleDiagnostics diagnostics;
    samples::control::RcControlState produced{};
    std::uint64_t next_log_ms = 0;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        (void)remote.snapshot(snapshot);
        const auto &rc = adapter.update(snapshot.remote, now);
        const auto diagnostic = diagnostics.update(now, topology.requested(),
            !rc.fresh || rc.remote.left_switch == robotics::RcSwitch::Down);
        if (!diagnostic.input_paused) produced = rc;
        produced.run_allowed = produced.run_allowed && rc.run_allowed;
        const bool source_fresh = produced.fresh && robotics::isFresh(produced.remote.stamp, now, 100);
        if (rc.run_allowed && !source_fresh) adapter.withdraw();
        topology.updateRemote(produced, source_fresh);
        if (!diagnostic.execution_paused) topology.tick();
        if (now >= next_log_ms) { next_log_ms = now + 500; topology.logStatus(); }
        k_sleep(K_MSEC(kControlPeriodMs));
    }
}
