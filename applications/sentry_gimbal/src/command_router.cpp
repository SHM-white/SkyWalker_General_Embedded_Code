#include "command_router.hpp"
RouteReport CommandRouter::route(const skywalker::robotics::RobotCommand &command) {
    const int result = local_.put(command.gimbal);
    remote_.submit(command.chassis);
    return {result};
}
