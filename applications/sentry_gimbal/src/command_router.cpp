#include "command_router.hpp"
RouteReport CommandRouter::route(const skywalker::robotics::RobotCommand &c,
                                 const skywalker::robotics::GlobalSafetyDecision &s,
                                 const skywalker::robotics::BoardHeartbeat &peer) {
    RouteReport report{};
    report.local_gimbal_result = local_.put({c.gimbal, s});
    skywalker::robotics::RemoteChassisControl remote{};
    remote.command = c.chassis;
    remote.global_action = s.chassis;
    remote.active_reasons = s.active_reasons;
    remote.receiver_boot_id = peer.sender_boot_id;
    remote.resume_generation = peer.resume_generation;
    remote.stamp = c.stamp;
    report.remote_chassis_result = remote_.put(remote);
    return report;
}
