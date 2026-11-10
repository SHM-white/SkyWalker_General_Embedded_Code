#pragma once
#include <zephyr/device.h>
#include <cstdint>
namespace bench {
inline const device *referee_uart = DEVICE_DT_GET(DT_ALIAS(referee_uart));
// Kconfig fixes the protocol at build time.
#if defined(CONFIG_SKYWALKER_REFEREE_RM2027)
inline constexpr const char *protocol_name = "RM2027 V1.0.0";
#elif defined(CONFIG_SKYWALKER_REFEREE_RM2026)
inline constexpr const char *protocol_name = "RM2026 V1.3.0";
#else
#error "Select a referee protocol in Kconfig"
#endif
inline constexpr std::uint32_t offline_timeout_ms = 500, permission_timeout_ms = 300;
}
