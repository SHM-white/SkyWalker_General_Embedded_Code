#include "../../common/shooter_bench.hpp"
int main() {
    return skywalker::samples::shooter::run(true, IS_ENABLED(CONFIG_SHOOTER_UNLOADED_FEED));
}
