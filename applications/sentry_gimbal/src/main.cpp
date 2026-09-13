#include <algorithm>
#include <cerrno>
#include <limits>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <communication/async_uart.hpp>
#include <communication/remote/remote_service.hpp>
#include <communication/referee/referee_service.hpp>
#include <communication/interboard/interboard_link.hpp>
#include <control/dji_motor_backend.hpp>
#include <control/dm_motor_backend.hpp>
#include <robotics/command/manual_command_mapper.hpp>
#include <robotics/command/command_manager.hpp>
#include <robotics/safety/global_safety_manager.hpp>
#include <robotics/safety/gimbal_local_safety.hpp>
#include "board_config.hpp"
#include "command_router.hpp"
LOG_MODULE_REGISTER(sentry_gimbal,LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;
namespace {
Latest<RemoteState> remote_state;
Latest<RefereeState> referee_state;
Latest<BoardHeartbeat> peer_heartbeat;
Latest<ChassisFeedbackSummary> chassis_feedback;
Latest<LocalGimbalCommand> local_command;
Latest<RemoteChassisControl> remote_command;
struct GimbalStatus { ExecutionState state=ExecutionState::Waiting; std::uint32_t generation=0,reasons=0; };
Latest<GimbalStatus> gimbal_status;
atomic_t reset_generation=0;

bool permission(const OutputPermission &p,std::uint64_t now) {
    if (p.valid) return p.enabled && isFresh(p.stamp,now,board_config::permission_timeout_ms);
    return !board_config::require_referee_for_motion;
}
std::uint32_t age(const MessageStamp &s,std::uint64_t now) {
    return s.valid && now>=s.timestamp_ms ? static_cast<std::uint32_t>(std::min<std::uint64_t>(now-s.timestamp_ms,UINT32_MAX)) : UINT32_MAX;
}
void remoteTask(void *,void *,void *) {
    static communication::AsyncUart uart(board_config::remote_uart);
    communication::RemoteService service({},{}); int ret=uart.init(); LOG_INF("remote UART init: %d",ret);
    for (;;) {
        const auto now=k_uptime_get(); uart.service(now);
        communication::AsyncUart::RxChunk chunk{};
        for (unsigned budget=0;budget<8;++budget) {
            ret=uart.read(chunk); if (ret==-EOVERFLOW) { service.discardPartial(); continue; } if (ret<0) break;
            service.processBytes(chunk.bytes,chunk.size,chunk.timestamp_ms);
        }
        RemoteState state{}; service.snapshot(now,state); remote_state.put(state); k_sleep(K_MSEC(1));
    }
}
void refereeTask(void *,void *,void *) {
    static communication::AsyncUart uart(board_config::referee_uart);
    communication::RefereeService service(board_config::referee_version);
    int ret=uart.init(); LOG_INF("referee UART init: %d, protocol profile: %d",ret,int(board_config::referee_version));
    for (;;) {
        const auto now=k_uptime_get(); uart.service(now); communication::AsyncUart::RxChunk chunk{};
        for (unsigned budget=0;budget<8;++budget) {
            ret=uart.read(chunk); if (ret==-EOVERFLOW) { service.discardPartial(); continue; } if (ret<0) break;
            service.processBytes(chunk.bytes,chunk.size,chunk.timestamp_ms);
        }
        service.processBytes(nullptr,0,now); RefereeState state{}; service.snapshot(now,state); referee_state.put(state); k_sleep(K_MSEC(1));
    }
}
void commandTask(void *,void *,void *) {
    ManualCommandMapper mapper({}); CommandManager manager({});
    GlobalSafetyManager safety({board_config::require_referee_for_motion,board_config::permission_timeout_ms});
    CommandRouter router(local_command,remote_command);
    RemoteState remote{}; RefereeState referee{}; BoardHeartbeat peer{};
    for (;;) {
        const auto now=k_uptime_get(); remote_state.get(remote); referee_state.get(referee); peer_heartbeat.get(peer);
        OperatorIntent intent{}; mapper.map(remote,intent);
        GlobalSafetyInputs input{}; input.now_ms=now;
        input.command_source_fresh=remote.online && isFresh(remote.stamp,now,board_config::command_timeout_ms);
        input.operator_motion_enabled=remote.stamp.valid && remote.left_switch==RcSwitch::Middle;
        input.emergency_stop_requested=board_config::emergencyStopRequested();
        input.gimbal_power=referee.robot.gimbal_output; input.chassis_power=referee.robot.chassis_output; input.shooter_power=referee.robot.shooter_output;
        if (board_config::takeEmergencyResetRequest() && safety.clearEmergencyStop(!input.emergency_stop_requested)==0) atomic_inc(&reset_generation);
        GlobalSafetyDecision decision{}; safety.evaluate(input,decision);
        if (!board_config::connections_configured) {
            decision.gimbal=decision.chassis=decision.shooter=SafetyAction::Disable;
            decision.state=SafetyState::ConfigBlocked; decision.active_reasons|=InvalidConfiguration;
        }
        RobotCommand command{};
        if (manager.step(intent,decision,now,command)==0) router.route(command,decision,peer);
        k_sleep(K_MSEC(10));
    }
}
void linkTask(void *,void *,void *) {
    static communication::AsyncUart uart(board_config::interboard_uart);
    communication::InterBoardLink link(BoardRole::GimbalController);
    int ret=uart.init(); LOG_INF("interboard UART init: %d",ret);
    const std::uint64_t boot_id=sys_rand64_get() | 1ULL;
    std::uint32_t heartbeat_sequence=0,control_sequence=0,constraint_sequence=0;
    std::uint64_t next_tx_ms=0,next_heartbeat_ms=0;
    RemoteChassisControl command{}; RefereeState referee{}; GimbalStatus status{};
    for (;;) {
        const auto now=static_cast<std::uint64_t>(k_uptime_get()); uart.service(now);
        communication::AsyncUart::RxChunk chunk{};
        for (unsigned budget=0;budget<8;++budget) {
            ret=uart.read(chunk); if (ret==-EOVERFLOW) { link.discardPartial(); continue; } if (ret<0) break;
            link.processRxBytes(chunk.bytes,chunk.size,chunk.timestamp_ms);
        }
        link.processRxBytes(nullptr,0,now);
        BoardHeartbeat peer{}; link.latestHeartbeat(peer); peer_heartbeat.put(peer);
        ChassisFeedbackSummary feedback{}; link.latestChassisFeedback(feedback); chassis_feedback.put(feedback);
        if (now>=next_tx_ms && !uart.txBusy()) {
            next_tx_ms=now+10; remote_command.get(command); referee_state.get(referee); gimbal_status.get(status);
            std::uint8_t bytes[256]{}; std::size_t used=0;
            auto append=[&](int n) { if (n>0) used+=n; };
            if (now>=next_heartbeat_ms) {
                next_heartbeat_ms=now+20; BoardHeartbeat h{}; h.role=BoardRole::GimbalController; h.sender_boot_id=boot_id;
                h.sender_uptime_ms=now; h.resume_generation=status.generation; h.ready=status.state==ExecutionState::Ready || status.state==ExecutionState::Active;
                h.safety_state=status.state==ExecutionState::ConfigBlocked ? SafetyState::ConfigBlocked : h.ready ? SafetyState::Ready : SafetyState::Waiting;
                h.active_reasons=status.reasons; h.sync_requested=!link.peerOnline(now);
                append(communication::InterBoardCodec::encodeHeartbeat(h,++heartbeat_sequence,bytes+used,sizeof(bytes)-used));
                ChassisConstraint constraint{}; constraint.output=referee.robot.chassis_output;
                constraint.output_age_ms=age(constraint.output.stamp,now);
                constraint.power_valid=referee.power.limit_stamp.valid && referee.power.stamp.valid;
                constraint.power_limit_w=referee.power.chassis_power_limit_w; constraint.buffer_energy_j=referee.power.buffer_energy_j;
                constraint.power_age_ms=std::max(age(referee.power.limit_stamp,now),age(referee.power.stamp,now));
                append(communication::InterBoardCodec::encodeChassisConstraint(constraint,++constraint_sequence,bytes+used,sizeof(bytes)-used));
            }
            // Preserve producer sequence/context. TX must never refresh an old command.
            if (command.stamp.valid) append(communication::InterBoardCodec::encodeChassisControl(command,++control_sequence,bytes+used,sizeof(bytes)-used));
            if (used) uart.send(bytes,used);
        }
        k_sleep(K_MSEC(1));
    }
}
void gimbalTask(void *,void *,void *) {
    static control::DjiMotorBackend dji_backend(board_config::yaw_motor);
    static control::DmMotorBackend dm_backend(board_config::yaw_motor);
    auto &backend=board_config::yaw_is_dm ? static_cast<control::MotorBackend &>(dm_backend) : static_cast<control::MotorBackend &>(dji_backend);
    static control::PositionMotor motor(backend,board_config::motorConfig());
    YawGimbal yaw(motor,board_config::yaw); GimbalLocalSafety local({board_config::command_timeout_ms});
    const int configured=board_config::connections_configured ? yaw.begin() : -ENODEV;
    LocalGimbalCommand command{}; RefereeState referee{}; std::uint32_t generation=0;
    std::uint64_t ready_ms=0,last_log=0; auto previous_ms=k_uptime_get(); atomic_val_t last_reset=0;
    for (;;) {
        const auto now=k_uptime_get(); const float dt=(now-previous_ms)/1000.0f; previous_ms=now;
        local_command.get(command); referee_state.get(referee);
        const bool estop=board_config::emergencyStopRequested() || (command.safety.active_reasons & EmergencyStop);
        const auto reset=atomic_get(&reset_generation);
        if (reset!=last_reset) { local.clearEmergencyStop(!estop); yaw.clearEmergencyStop(!estop); last_reset=reset; }
        const bool powered=permission(referee.robot.gimbal_output,now);
        if (configured==0) {
            if (estop) yaw.suspend(PauseReason::EmergencyStop);
            else if (!powered) { if (motor.state()!=ExecutionState::Waiting && motor.state()!=ExecutionState::EStopLatched) yaw.suspend(PauseReason::RefereeDisabled); }
            else if (motor.state()!=ExecutionState::Active) {
                if (yaw.poll(now)==0 && generation!=motor.status().resume_generation) { generation=motor.status().resume_generation; ready_ms=now; }
            }
        }
        const auto &fb=motor.telemetry().measurement.feedback;
        const std::uint32_t timeout=board_config::yaw_is_dm ? CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS : CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS;
        LocalSafetyInputs input{}; input.now_ms=now; input.config_valid=configured==0; input.power_allowed=powered;
        input.emergency_stop_requested=estop; input.hardware_ready=motor.state()==ExecutionState::Ready || motor.state()==ExecutionState::Active;
        input.armed=motor.state()==ExecutionState::Active;
        input.feedback_fresh=fb.timestamp_ms && now>=static_cast<std::int64_t>(fb.timestamp_ms) && std::uint64_t(now)-fb.timestamp_ms<=timeout;
        input.command_stamp=command.command.stamp;
        input.global_action=isFresh(command.safety.stamp,now,board_config::command_timeout_ms) ? command.safety.gimbal : SafetyAction::Disable;
        LocalSafetyDecision decision{}; local.evaluate(input,decision);
        if (configured==0) {
            if (decision.action==SafetyAction::Disable) {
                if (motor.state()==ExecutionState::Active) yaw.suspend(PauseReason::CommandTimeout);
            } else if (motor.state()==ExecutionState::Active || command.command.stamp.timestamp_ms>=ready_ms) {
                GimbalCommand target=command.command;
                if (decision.action==SafetyAction::Hold) target.mode=GimbalMode::Hold;
                yaw.update(target,decision.action,dt);
            }
        }
        gimbal_status.put({configured<0 ? ExecutionState::ConfigBlocked : motor.state(),generation,decision.active_reasons});
        if (std::uint64_t(now)>=last_log+1000) {
            last_log=now; LOG_INF("uptime=%lld yaw=%d reasons=%x gen=%u error=%d",now,configured<0 ? int(ExecutionState::ConfigBlocked) : int(motor.state()),decision.active_reasons,generation,configured<0 ? configured : motor.status().last_recovery_error);
        }
        k_sleep(K_MSEC(5));
    }
}
}
K_THREAD_DEFINE(remote_thread,3072,remoteTask,nullptr,nullptr,nullptr,6,0,0);
K_THREAD_DEFINE(referee_thread,6144,refereeTask,nullptr,nullptr,nullptr,6,0,0);
K_THREAD_DEFINE(link_thread,8192,linkTask,nullptr,nullptr,nullptr,5,0,0);
K_THREAD_DEFINE(command_thread,4096,commandTask,nullptr,nullptr,nullptr,5,0,0);
K_THREAD_DEFINE(gimbal_thread,6144,gimbalTask,nullptr,nullptr,nullptr,4,0,0);
int main() {
    LOG_INF("MC02 sentry gimbal: configured=%d referee_required=%d; edit app.overlay and src/board_config.hpp",board_config::connections_configured,board_config::require_referee_for_motion);
    return 0; // Worker threads and all callback-owned objects remain alive.
}
