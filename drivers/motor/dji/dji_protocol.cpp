#include <drivers/motor/dji_protocol.hpp>

namespace skywalker::motor::dji {

    static inline std::uint16_t readBe16(const std::uint8_t *p) {
        return (static_cast<std::uint16_t>(p[0]) << 8) | static_cast<std::uint16_t>(p[1]);
    }

}

