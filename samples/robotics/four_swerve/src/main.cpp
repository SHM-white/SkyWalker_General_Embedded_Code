#include "board_config.hpp"
#include "../../common/chassis_bench.hpp"

int main() {
    return skywalker::samples::chassis::run(bench::steer_can, bench::drive_can, false);
}
