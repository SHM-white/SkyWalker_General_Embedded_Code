#include "../../common/shooter_bench.hpp"
int main() {
    return skywalker::samples::shooter::run(false, IS_ENABLED(CONFIG_SHOOTER_UNLOADED_FEED));
}
