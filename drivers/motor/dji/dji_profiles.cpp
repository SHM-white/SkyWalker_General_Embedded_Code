#include <cmath>
#include <errno.h>

#include "dji_internal.hpp"

namespace skywalker::motor::dji::internal
{

    const Profile kM3508C620Profile = {
        Model::M3508C620,
        8u,
        0x200u,
        0x200u,
        0x1FFu,
        16384,
        20.0f,
        true,
    };

    const Profile kM2006C610Profile = {
        Model::M2006C610,
        8u,
        0x200u,
        0x200u,
        0x1FFu,
        10000,
        10.0f,
        false,
    };

    const Profile kGM6020CurrentProfile = {
        Model::GM6020Current,
        7u,
        0x204u,
        0x1FEu,
        0x2FEu,
        16384,
        3.0f,
        true,
    };

    int resolveEndpoint(const Profile &profile, std::uint8_t motor_id, Endpoint &out)
    {
        if (motor_id == 0u || motor_id > profile.max_motor_id)
        {
            return -ERANGE;
        }

        Endpoint next{};
        next.feedback_id = static_cast<std::uint16_t>(profile.feedback_base + motor_id);

        if (motor_id <= 4u)
        {
            next.command_id = profile.low_command_id;
            next.command_slot = static_cast<std::uint8_t>(motor_id - 1u);
        }
        else
        {
            next.command_id = profile.high_command_id;
            next.command_slot = static_cast<std::uint8_t>(motor_id - 5u);
        }

        if (next.command_slot >= 4u)
            return -ERANGE;
        out = next;
        return 0;
    }

    int currentToRaw(const Profile &profile, float current_a, std::int16_t &out)
    {
        if (!std::isfinite(current_a))
            return -EINVAL;
        if (profile.command_raw_max <= 0 || !std::isfinite(profile.protocol_current_max_a) 
            || profile.protocol_current_max_a <= 0.0f)
        {
            return -EINVAL;
        }
        if (std::fabs(current_a) > profile.protocol_current_max_a)
        {
            return -ERANGE;
        }

        const float scaled = current_a * static_cast<float>(profile.command_raw_max) / profile.protocol_current_max_a;
        const long rounded = std::lround(scaled);
        if (rounded < -profile.command_raw_max || rounded > profile.command_raw_max)
        {
            return -ERANGE;
        }

        out = static_cast<std::int16_t>(rounded);
        return 0;
    }

    int rawToCurrent(const Profile &profile, std::int16_t raw, float &out)
    {
        if (profile.command_raw_max <= 0 || !std::isfinite(profile.protocol_current_max_a) || profile.protocol_current_max_a <= 0.0f)
        {
            return -EINVAL;
        }
        if (raw < -profile.command_raw_max || raw > profile.command_raw_max)
        {
            return -ERANGE;
        }

        out = static_cast<float>(raw) * profile.protocol_current_max_a / static_cast<float>(profile.command_raw_max);
        return 0;
    }

} // namespace skywalker::motor::dji::internal
