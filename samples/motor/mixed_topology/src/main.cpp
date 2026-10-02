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
constexpr motor::Timing kDjiTiming{20, 20, 30, 100};
constexpr motor::Timing kDmTiming{50, 20, 50, 3000};
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
constexpr const char *kTopologyName = "M3508 CAN1 + DM MIT CAN2, one linked Group";
#elif defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
constexpr const char *kTopologyName = "linked M3508/DM across CAN1/CAN2 plus one independent Group per CAN";
#else
#error "Select a MIXED_TOPOLOGY in CMakeLists.txt"
#endif

using samples::control::RcControlAdapter;
using samples::control::DiagnosticScenario;
static_assert(samples::control::diagnostic_scenario == DiagnosticScenario::None ||
              samples::control::diagnostic_scenario == DiagnosticScenario::InputPause ||
              samples::control::diagnostic_scenario == DiagnosticScenario::ExecutionPause);

// Each independently faulted group requires its own released channel before
// another rising request. A failed group never withdraws another group's RC.
struct GroupRequest {
    bool released = false;
    bool issued = false;
    void update(motor::Group &group, bool allowed, float channel, bool safe, bool clear) {
        auto state = group.status();
        if (clear) {
            group.disable();
            const int ret = group.clearFault();
            LOG_INF("group clear=%d", ret);
            issued = released = false;
            return;
        }
        if (!allowed) {
            if (state.active || state.enable_pending) group.disable();
            issued = false;
            released = channel <= .3f;
            return;
        }
        if (issued && !state.active && !state.enable_pending) {
            issued = false;
            released = false;
        }
        if (channel <= .3f) {
            if (state.active || state.enable_pending) group.disable();
            issued = false;
            released = true;
        } else if (channel > .6f && released && !issued) {
            released = false;
            const int ret = safe ? group.enable() : -EAGAIN;
            issued = ret == 0;
            LOG_INF("group enable=%d", ret);
        }
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

    static bool safeToEnable(const motor::Motor &drive) {
        const auto view = drive.snapshot();
        if (!drive.ready() || !view.feedback_fresh || (view.feedback.valid & motor::FeedbackVelocity) == 0u ||
            !std::isfinite(view.feedback.velocity_rad_s) || std::fabs(view.feedback.velocity_rad_s) >= 10.0f)
            return false;
        if ((drive.info().capabilities & motor::FeedbackTemperature) != 0u &&
            (((view.feedback.valid & motor::FeedbackTemperature) == 0u) ||
             !std::isfinite(view.feedback.temperature_c) || view.feedback.temperature_c >= 60.0f))
            return false;
        if ((drive.info().capabilities & motor::CommandTorque) != 0u && !view.native_temperatures_valid)
            return false;
        return !view.native_temperatures_valid ||
               (std::isfinite(view.native_mos_temperature_c) && std::isfinite(view.native_rotor_temperature_c) &&
                view.native_mos_temperature_c < 60.0f && view.native_rotor_temperature_c < 60.0f);
    }

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

    void updateRemote(const samples::control::RcControlState &rc, bool source_fresh) {
        const bool allowed = rc.run_allowed && source_fresh;
        const float first_channel = RcControlAdapter::normalize(rc.remote.analog.left_y);
        const float second_channel = RcControlAdapter::normalize(rc.remote.analog.right_y);
        const float third_channel = RcControlAdapter::normalize(rc.remote.analog.wheel);
#if defined(MIXED_TOPOLOGY_CROSS_CAN_GROUP) || defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        request1.update(linked_group, allowed, first_channel,
                        safeToEnable(first) && safeToEnable(second), rc.clear_fault);
#if defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        request2.update(third_group, allowed, second_channel, safeToEnable(third), rc.clear_fault);
        request3.update(fourth_group, allowed, third_channel, safeToEnable(fourth), rc.clear_fault);
#else
        (void)second_channel; (void)third_channel;
#endif
#else
        request1.update(first_group, allowed, first_channel, safeToEnable(first), rc.clear_fault);
        request2.update(second_group, allowed, second_channel, safeToEnable(second), rc.clear_fault);
        (void)third_channel;
#endif
    }

    bool active() const {
#if defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        return linked_group.active() || third_group.active() || fourth_group.active();
#elif defined(MIXED_TOPOLOGY_CROSS_CAN_GROUP)
        return linked_group.active();
#else
        return first_group.active() || second_group.active();
#endif
    }

    void tick() {
#if defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        bool can1_command = false;
        bool can2_command = false;
        if (linked_group.active()) {
            int ret = first.setCurrent(kDjiTestCurrentA);
            if (ret == 0)
                ret = second.setTorque(kDmTestTorqueNm);
            if (ret < 0) {
                linked_group.disable();
                LOG_ERR("linked command rejected: %d", ret);
            }
            else {
                can1_command = true;
                can2_command = true;
            }
        }
        if (third_group.active()) {
            const int ret = third.setCurrent(kDjiTestCurrentA);
            if (ret < 0) {
                third_group.disable();
                LOG_ERR("CAN1 independent command rejected: %d", ret);
            }
            else {
                can1_command = true;
            }
        }
        if (fourth_group.active()) {
            const int ret = fourth.setTorque(kDmTestTorqueNm);
            if (ret < 0) {
                fourth_group.disable();
                LOG_ERR("CAN2 independent command rejected: %d", ret);
            }
            else {
                can2_command = true;
            }
        }
        if (can1_command) {
            const auto result = can1.commit();
            if (result.error < 0) {
                linked_group.disable();
                third_group.disable();
                LOG_ERR("CAN1 commit failed: %d", result.error);
            }
        }
        if (can2_command) {
            const auto result = can2.commit();
            if (result.error < 0) {
                linked_group.disable();
                fourth_group.disable();
                LOG_ERR("CAN2 commit failed: %d", result.error);
            }
        }
#elif defined(MIXED_TOPOLOGY_CROSS_CAN_GROUP)
        if (!linked_group.active())
            return;
        int ret = first.setCurrent(kDjiTestCurrentA);
        if (ret == 0)
            ret = second.setTorque(kDmTestTorqueNm);
        if (ret < 0) {
            linked_group.disable();
            LOG_ERR("linked command rejected: %d", ret);
            return;
        }
        const auto first_commit = can1.commit();
        const auto second_commit = can2.commit();
        if (first_commit.error < 0 || second_commit.error < 0) {
            linked_group.disable();
            LOG_ERR("linked commit failed: %d / %d", first_commit.error, second_commit.error);
        }
#else
        const bool first_active = first_group.active();
        const bool second_active = second_group.active();
        if (!first_active && !second_active)
            return;
        if (first_active) {
#if defined(MIXED_TOPOLOGY_DJI_SHARED_FRAME)
            const int ret = first.setCurrent(kDjiTestCurrentA);
#else
            const int ret = first.setTorque(kDmTestTorqueNm);
#endif
            if (ret < 0) {
                first_group.disable();
                LOG_ERR("motor 1 command rejected: %d", ret);
            }
        }
        if (second_active) {
#if defined(MIXED_TOPOLOGY_DJI_SHARED_FRAME)
            const int ret = second.setCurrent(kDjiTestCurrentA);
#else
            const int ret = second.setTorque(kDmTestTorqueNm);
#endif
            if (ret < 0) {
                second_group.disable();
                LOG_ERR("motor 2 command rejected: %d", ret);
            }
        }
        const auto result = can1.commit();
        if (result.error < 0) {
            first_group.disable();
            second_group.disable();
            LOG_ERR("CAN1 commit failed: %d", result.error);
        }
#endif
    }

    static void logMotor(const char *name, const motor::Motor &drive) {
        const auto view = drive.snapshot();
        LOG_INF("%s state=%u fresh=%d gen=%llu ref=%llu feedback_ms=%llu stop=%u/%llu/%d fault=%u/%d", name,
                unsigned(view.state), view.feedback_fresh, view.enable_generation, view.reference_generation,
                view.feedback.timestamp_ms, unsigned(view.stop.progress), view.stop.request_generation,
                view.stop.tx_error, unsigned(view.last_fault.reason), view.last_fault.error);
    }

    static void logBus(const char *name, const motor::CanBus &bus) {
        const auto status = bus.status();
        LOG_INF("%s state=%u commit=%llu last_tx=%d/%llu/0x%03x/%u/%d/%llu error=%d", name, unsigned(status.state),
                status.latest_submitted_sequence, status.last_tx.valid, status.last_tx.sequence,
                unsigned(status.last_tx.can_id), unsigned(status.last_tx.purpose), status.last_tx.error,
                status.last_tx.completed_ms, status.last_error);
    }

    void logStatus() const {
        LOG_INF("uptime_ms=%lld topology=%s", k_uptime_get(), kTopologyName);
#if defined(MIXED_TOPOLOGY_CROSS_CAN_GROUP) || defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        const auto linked = linked_group.status();
        LOG_INF("linked ready=%d active=%d pending=%d gen=%llu fault=%u/%d", linked.ready, linked.active,
                linked.enable_pending, linked.enable_generation, unsigned(linked.last_fault.reason),
                linked.last_fault.error);
        logMotor("linked CAN1 M3508", first);
        logMotor("linked CAN2 DM", second);
#if defined(MIXED_TOPOLOGY_CROSS_CAN_ISOLATION)
        const auto can1_group = third_group.status();
        const auto can2_group = fourth_group.status();
        LOG_INF("independent CAN1=%d/%d/%llu CAN2=%d/%d/%llu", can1_group.ready, can1_group.active,
                can1_group.enable_generation, can2_group.ready, can2_group.active, can2_group.enable_generation);
        logMotor("independent CAN1 M2006", third);
        logMotor("independent CAN2 DM", fourth);
#endif
        logBus("CAN1", can1);
        logBus("CAN2", can2);
#else
        const auto group_a = first_group.status();
        const auto group_b = second_group.status();
        LOG_INF("G1 ready=%d active=%d pending=%d gen=%llu G2 ready=%d active=%d pending=%d gen=%llu", group_a.ready,
                group_a.active, group_a.enable_pending, group_a.enable_generation, group_b.ready, group_b.active,
                group_b.enable_pending, group_b.enable_generation);
        logMotor("G1", first);
        logMotor("G2", second);
        logBus("CAN1", can1);
#endif
    }
};

} // namespace

int main() {
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                                 samples::control::receiverConfig());
    const int remote_error = remote.start();
    if (remote_error < 0) return remote_error;
    static Topology topology{};
    LOG_INF("topology: %s", kTopologyName);
    const int ret = topology.start();
    if (ret < 0) { LOG_ERR("topology start failed: %d", ret); return ret; }
    LOG_INF("RC: safe+center 0.5s then left Middle; left/right Y and wheel hold groups, safe stops all");
    communication::RemoteReceiver::Snapshot snapshot{};
    RcControlAdapter adapter;
    samples::control::SampleDiagnostics diagnostics;
    samples::control::RcControlState produced{};
    std::int64_t next_log_ms = 0;
    for (;;) {
        const auto now_ms = static_cast<std::uint64_t>(k_uptime_get());
        (void)remote.snapshot(snapshot);
        const auto &rc = adapter.update(snapshot.remote, now_ms);
        const auto diagnostic = diagnostics.update(now_ms, topology.active(),
            !rc.fresh || rc.remote.left_switch == robotics::RcSwitch::Down);
        if (!diagnostic.input_paused) produced = rc;
        // Explicit stop/clear bypass the paused producer and retain live RC ownership.
        produced.run_allowed = produced.run_allowed && rc.run_allowed;
        produced.clear_fault = rc.clear_fault;
        const bool source_fresh = produced.fresh && robotics::isFresh(produced.remote.stamp, now_ms, 100);
        if (rc.run_allowed && !source_fresh) adapter.withdraw();
        topology.updateRemote(produced, source_fresh);
        if (!diagnostic.execution_paused) topology.tick();
        if (now_ms >= static_cast<std::uint64_t>(next_log_ms)) {
            topology.logStatus();
            next_log_ms = now_ms + 500;
        }
        k_sleep(K_MSEC(kControlPeriodMs));
    }
}
