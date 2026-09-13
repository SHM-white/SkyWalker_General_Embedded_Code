#pragma once
#include "latest.hpp"
#include <robotics/messages/safety.hpp>
#include <robotics/messages/interboard.hpp>
struct LocalGimbalCommand { skywalker::robotics::GimbalCommand command{}; skywalker::robotics::GlobalSafetyDecision safety{}; };
struct RouteReport { int local_gimbal_result=0,remote_chassis_result=0; };
class CommandRouter {
public:
    CommandRouter(Latest<LocalGimbalCommand> &local,Latest<skywalker::robotics::RemoteChassisControl> &remote): local_(local),remote_(remote) {}
    RouteReport route(const skywalker::robotics::RobotCommand &,const skywalker::robotics::GlobalSafetyDecision &,const skywalker::robotics::BoardHeartbeat &peer);
private: Latest<LocalGimbalCommand> &local_; Latest<skywalker::robotics::RemoteChassisControl> &remote_;
};
