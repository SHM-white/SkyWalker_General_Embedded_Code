#pragma once
#include <latest.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
struct RouteReport { int local_gimbal_result = 0; };
class CommandRouter {
public:
    CommandRouter(Latest<skywalker::robotics::GimbalCommand> &local,
                  skywalker::communication::InterBoardEndpoint &remote) : local_(local), remote_(remote) {}
    RouteReport route(const skywalker::robotics::RobotCommand &);
private:
    Latest<skywalker::robotics::GimbalCommand> &local_;
    skywalker::communication::InterBoardEndpoint &remote_;
};
