#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/pid.h>
#include <control/slew_rate_limiter.h>
#include <drivers/motor/dji_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(dji_position_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

namespace {

constexpr std::int64_t kControlPeriodMs = 5;
/* One telemetry frame per five control cycles: 200 Hz. */
constexpr std::uint32_t kTelemetryPeriodCycles = 5U;
constexpr float kTargetOffsetRad = 3.0f;

/*
 * ======================== 串级位置环调参区 ========================
 *
 * 控制结构只有两层：
 *   位置误差 --位置P--> 目标速度 --斜坡限制--> 速度误差 --速度PI--> 电流
 *
 * 建议严格按下面的顺序调，一次只改一个量：
 *
 * 1. kVelocityKp：先把 kVelocityKi 保持为 0，只调速度 P。
 *    - 增大：速度跟随更快、受负载后掉速更少，但过大会啸叫、抖动，
 *      还会放大 GM6020 速度反馈的量化台阶。
 *    - 减小：运行更柔和，但速度误差变大，严重时电流不足以克服摩擦。
 *
 * 2. kVelocityKi：速度 P 已稳定后再从 0 小幅增加。
 *    - 增大：逐渐补足摩擦和恒定负载需要的电流，消除稳态速度误差；
 *    - 过大：容易低频来回摆动、越过目标后回拉，甚至长时间顶住限流。
 *    速度 PI 已经能处理通常的静摩擦和运行摩擦，因此这里不再额外提供
 *    “静态摩擦电流/动态摩擦电流”等前馈参数。
 *
 * 3. kPositionKp：速度内环稳定后再调位置 P。
 *    - 增大：相同位置误差会要求更高速度，到位更快；
 *    - 过大：更早撞上最大速度，且接近目标时容易过冲、反复换向；
 *    - 减小：动作更慢、更柔和。位置环一般不需要 I，恒定摩擦交给
 *      速度 PI 处理，所以位置 Ki 固定为 0，不作为调参项。
 *
 * 4. kVelocityRampRateRadS2：最后调整目标速度每秒最多变化多少。
 *    - 增大：启动和制动更干脆，但机械冲击和所需峰值电流增加；
 *    - 减小：动作更平滑，但制动过慢时可能越过目标后才开始回拉。
 *
 * 当前数值保留自工作区原调参结果；它们只是起点，不代表已在实物上
 * 验证稳定。尤其是 5 rad/s 和 25 rad/s^2，带载测试前必须确认安全。
 */
constexpr float kPositionKp = 10.0f;
constexpr float kVelocityKp = 0.04f;
constexpr float kVelocityKi = 0.0f;
constexpr float kVelocityRampRateRadS2 = 25.0f;

/*
 * ======================== 边界与安全参数 ========================
 *
 * 这些值用于规定“最多允许多快、多大电流、到多近算到位”，不要和 PID
 * 增益一起反复扫参。只有机械允许范围、供电能力或验收精度改变时才调整。
 *
 * kVelocityAbsMaxRadS：位置外环能够请求的最大速度。
 *   它决定长距离运动的速度上限，不负责解决摩擦或到位慢。提高前先确认
 *   机构、供电和急停能力，并保证速度 PI 在较低速度下已经稳定。
 *
 * kSoftwareCurrentAbsMaxA：速度 PI 能输出的最大正/负电流。
 *   它直接限制最大转矩和加减速度。若电机不起转，应先确认方向、卡滞和
 *   速度 PI，再依据实测起转电流小步提高，不能靠一次性放大来“治”误差。
 *   速度积分项也复用这个范围，因此不再保留单独的积分限幅调参项。
 *
 * kPositionDeadbandRad：位置误差绝对值小于它时视为到位。
 *   增大可减少编码器量化和摩擦导致的目标附近反复动作，但最终允许误差
 *   也会增大；减小可提高理论精度，但太小会持续抖动。应最后再调。
 *
 * 实测超速阈值由最大命令速度自动取 1.5 倍，不再单独调参。若实测速度
 * 超过它，说明方向、负载、速度环或反馈可能异常，程序立即停机。
 */
constexpr float kVelocityAbsMaxRadS = 5.0f;
constexpr float kSoftwareCurrentAbsMaxA = 0.10f;
constexpr float kPositionDeadbandRad = 0.012f;
constexpr float kMeasuredVelocitySafetyMaxRadS = 1.5f * kVelocityAbsMaxRadS;

struct PositionController {
    control_pid_config position_config{};
    control_pid_state position_state{};
    control_slew_rate_config velocity_reference_config{};
    control_slew_rate_state velocity_reference_state{};
    control_pid_config velocity_config{};
    control_pid_state velocity_state{};
};

struct PositionControlOutput {
    control_pid_result position{};
    float velocity_reference_rad_s = 0.0f;
    float acceleration_reference_rad_s2 = 0.0f;
    control_pid_result velocity{};
    float current_command_a = 0.0f;
};

skywalker::motor::dji::Bus dji_bus;
volatile std::int32_t runtime_diagnostic = 0;

PositionController makePositionController() {
    PositionController controller{};

    /* Position error (rad) -> velocity request (rad/s). */
    controller.position_config = {
        .kp = kPositionKp,
        .ki = 0.01f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = 0.8f,
        .integral_max = 0.8f,
        .output_min = -kVelocityAbsMaxRadS,
        .output_max = kVelocityAbsMaxRadS,
        .deadband = kPositionDeadbandRad,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };

    controller.velocity_reference_config = {
        /* 正向、反向共用一个变化率，避免分别维护两套斜坡参数。 */
        .rising_rate_per_s = kVelocityRampRateRadS2,
        .falling_rate_per_s = kVelocityRampRateRadS2,
    };

    /*
     * Velocity error (rad/s) -> current command (A).
     * I 项和最终输出共用软件电流边界；PID 内部已有抗积分饱和逻辑。
     */
    controller.velocity_config = {
        .kp = kVelocityKp,
        .ki = kVelocityKi,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = -kSoftwareCurrentAbsMaxA,
        .integral_max = kSoftwareCurrentAbsMaxA,
        .output_min = -kSoftwareCurrentAbsMaxA,
        .output_max = kSoftwareCurrentAbsMaxA,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };

    return controller;
}

int validateController(const PositionController &controller) {
    int ret = control_pid_validate(&controller.position_config);
    if (ret < 0) {
        return ret;
    }
    ret = control_slew_rate_validate(&controller.velocity_reference_config);
    if (ret < 0) {
        return ret;
    }
    return control_pid_validate(&controller.velocity_config);
}

int waitForFreshFeedback(const struct device *motor) {
    const std::int64_t deadline_ms = k_uptime_get() + 2000;

    while (skywalker::motor::getState(motor) != skywalker::motor::State::Ready) {
        if (k_uptime_get() >= deadline_ms) {
            return -ETIMEDOUT;
        }
        k_sleep(K_MSEC(5));
    }
    return 0;
}

int readFreshPositionFeedback(const struct device *motor, std::uint64_t now_ms, skywalker::motor::Feedback &feedback) {
    int ret = skywalker::motor::readFeedback(motor, feedback);
    if (ret < 0) {
        return ret;
    }
    if (skywalker::motor::getState(motor) != skywalker::motor::State::Ready) {
        return -EHOSTDOWN;
    }

    constexpr std::uint32_t required = skywalker::motor::FeedbackPosition | skywalker::motor::FeedbackVelocity;
    if ((feedback.valid & required) != required) {
        return -ENODATA;
    }
    if (!std::isfinite(feedback.position_rad) || !std::isfinite(feedback.velocity_rad_s)) {
        return -EINVAL;
    }
    if (feedback.timestamp_ms == 0U || now_ms < feedback.timestamp_ms || now_ms - feedback.timestamp_ms > CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS) {
        return -ESTALE;
    }
    return 0;
}

int resetController(PositionController &controller, const skywalker::motor::Feedback &feedback) {
    int ret = control_pid_reset(&controller.position_state, feedback.position_rad);
    if (ret < 0) {
        return ret;
    }
    ret = control_slew_rate_reset(&controller.velocity_reference_state, 0.0f);
    if (ret < 0) {
        return ret;
    }
    return control_pid_reset(&controller.velocity_state, feedback.velocity_rad_s);
}

int calculatePositionCurrent(PositionController &controller, const skywalker::motor::Feedback &feedback, float position_target_rad, float dt_s,
                             PositionControlOutput &output) {
    if (!std::isfinite(position_target_rad) || !std::isfinite(dt_s)) {
        return -EINVAL;
    }

    control_pid_state next_position_state = controller.position_state;
    control_slew_rate_state next_reference_state = controller.velocity_reference_state;
    control_pid_state next_velocity_state = controller.velocity_state;
    PositionControlOutput next_output{};

    const control_pid_input position_input = {
        .setpoint = position_target_rad,
        .measurement = feedback.position_rad,
        .dt_s = dt_s,
        /* 位置环固定为 P-only，没有需要更新的积分状态。 */
        .freeze_integrator = true,
    };
    int ret = control_pid_step(&next_position_state, &controller.position_config, &position_input, &next_output.position);
    if (ret < 0) {
        return ret;
    }

    ret = control_slew_rate_step(&next_reference_state, &controller.velocity_reference_config, next_output.position.output, dt_s,
                                 &next_output.velocity_reference_rad_s, &next_output.acceleration_reference_rad_s2);
    if (ret < 0) {
        return ret;
    }

    const control_pid_input velocity_input = {
        .setpoint = next_output.velocity_reference_rad_s,
        .measurement = feedback.velocity_rad_s,
        .dt_s = dt_s,
        .freeze_integrator = false,
    };
    ret = control_pid_step(&next_velocity_state, &controller.velocity_config, &velocity_input, &next_output.velocity);
    if (ret < 0) {
        return ret;
    }

    /* 速度 PID 的 output 已经做过正负软件电流限幅，可直接作为电流命令。 */
    next_output.current_command_a = next_output.velocity.output;
    if (!std::isfinite(next_output.current_command_a)) {
        return -ERANGE;
    }

    controller.position_state = next_position_state;
    controller.velocity_reference_state = next_reference_state;
    controller.velocity_state = next_velocity_state;
    output = next_output;
    return 0;
}

int stopAfterFailure(int original_error) {
    runtime_diagnostic = original_error;
    skywalker::motor::dji::FlushReport report{};
    const int stop_ret = dji_bus.stop(report);
    LOG_ERR("control failed: cause=%d stop=%d zero=%d zero_err=%d", original_error, stop_ret, report.zero_sent ? 1 : 0, report.zero_tx_error);
    return stop_ret < 0 ? stop_ret : original_error;
}

/*
 * Position trajectory (rad). Returns where the output shaft should be at
 * elapsed_ms. Edit this function to change the motion profile without
 * touching the control loop: 5 s settle at the start position, then a
 * step to the final target held forever.
 */
float requestedPositionRad(std::int64_t elapsed_ms, float initial_position_rad, float final_target_rad) {
    if (elapsed_ms < 5000) {
        return initial_position_rad;
    }
    int modded_ms = elapsed_ms % 40000;
    if (modded_ms < 10000) {
        return final_target_rad;
    } else if (modded_ms < 20000) {
        return final_target_rad * 2;
    } else if (modded_ms < 30000) {
        return final_target_rad * 3;
    } else
        return final_target_rad * 4;
}

} // namespace

int main() {
    runtime_diagnostic = 1;
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    const struct device *vofa_uart = DEVICE_DT_GET(DT_NODELABEL(usart6));
    if (!device_is_ready(motor)) {
        LOG_ERR("motor device not ready");
        return -ENODEV;
    }
    if (!device_is_ready(vofa_uart)) {
        LOG_ERR("VOFA UART device not ready");
        return -ENODEV;
    }

    Vofa vofa{};
    vofa_init(&vofa, vofa_uart);

    skywalker::motor::dji::Descriptor descriptor{};
    int ret = skywalker::motor::dji::describe(motor, descriptor);
    if (ret < 0 || descriptor.can == nullptr || !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }
    runtime_diagnostic = 10;

    PositionController controller = makePositionController();
    ret = validateController(controller);
    if (ret < 0) {
        LOG_ERR("controller config invalid: %d", ret);
        return ret;
    }
    runtime_diagnostic = 11;

    ret = dji_bus.init(descriptor.can);
    if (ret < 0) {
        LOG_ERR("Bus init failed: %d", ret);
        return ret;
    }
    runtime_diagnostic = 12;
    ret = dji_bus.attach(motor);
    if (ret < 0) {
        LOG_ERR("Bus attach failed: %d", ret);
        return ret;
    }
    runtime_diagnostic = 13;
    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("no fresh feedback before arm: %d", ret);
        return ret;
    }
    runtime_diagnostic = 2;

    LOG_INF("feedback ready: arming now; keep the GM6020 output "
            "suspended and hold a power cut");

    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("feedback lost before arm: %d", ret);
        return ret;
    }

    skywalker::motor::Feedback first_feedback{};
    const std::uint64_t reset_time_ms = static_cast<std::uint64_t>(k_uptime_get());
    ret = readFreshPositionFeedback(motor, reset_time_ms, first_feedback);
    if (ret < 0) {
        LOG_ERR("initial feedback invalid: %d", ret);
        return ret;
    }
    ret = resetController(controller, first_feedback);
    if (ret < 0) {
        LOG_ERR("controller reset failed: %d", ret);
        return ret;
    }

    /* Feedback position is continuous; this sample commands a relative move. */
    const float initial_position_rad = first_feedback.position_rad;
    const float final_target_rad = initial_position_rad + kTargetOffsetRad;

    skywalker::motor::dji::FlushReport arm_report{};
    ret = dji_bus.arm(arm_report);
    if (ret < 0 || !arm_report.zero_sent) {
        LOG_ERR("arm/zero failed: ret=%d zero=%d zero_err=%d", ret, arm_report.zero_sent ? 1 : 0, arm_report.zero_tx_error);
        return ret < 0 ? ret : -EIO;
    }
    runtime_diagnostic = 3;

    const std::int64_t run_start_ms = k_uptime_get();
    std::int64_t previous_cycle_ms = run_start_ms;
    std::uint32_t telemetry_divider = 0;

    /* Run forever; pull power or reset to stop. */
    for (;;) {
        runtime_diagnostic = 100;
        k_sleep(K_MSEC(kControlPeriodMs));

        const std::int64_t now_signed_ms = k_uptime_get();
        if (now_signed_ms <= previous_cycle_ms) {
            return stopAfterFailure(-ERANGE);
        }
        const float dt_s = static_cast<float>(now_signed_ms - previous_cycle_ms) / 1000.0f;
        previous_cycle_ms = now_signed_ms;
        const std::uint64_t now_ms = static_cast<std::uint64_t>(now_signed_ms);

        skywalker::motor::Feedback feedback{};
        ret = readFreshPositionFeedback(motor, now_ms, feedback);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }
        // if (std::fabs(feedback.velocity_rad_s) > kMeasuredVelocitySafetyMaxRadS) {
        //     return stopAfterFailure(-ERANGE);
        // }

        const float target_rad = requestedPositionRad(now_signed_ms - run_start_ms, initial_position_rad, final_target_rad);

        PositionControlOutput output{};
        ret = calculatePositionCurrent(controller, feedback, target_rad, dt_s, output);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        ret = skywalker::motor::setCurrent(motor, output.current_command_a);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        skywalker::motor::dji::FlushReport flush_report{};
        ret = dji_bus.flush(flush_report);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        if (++telemetry_divider >= kTelemetryPeriodCycles) {
            telemetry_divider = 0U;
            /*
             * JustFloat 调参通道（顺序固定，便于直接对照曲线）：
             *  0~2：位置目标、位置反馈、位置误差；
             *  3~5：速度目标、速度反馈、速度误差；
             *  6~8：速度 P、速度 I、最终电流；
             *  9~10：本次控制周期 ms、反馈年龄 ms。
             *
             * 调速度环时重点看 3~8；速度环稳定后，调位置环再看 0~5。
             */
            const float channels[11] = {
                target_rad,
                feedback.position_rad,
                output.position.error,
                output.velocity_reference_rad_s,
                feedback.velocity_rad_s,
                output.velocity.error,
                output.velocity.p,
                output.velocity.i,
                output.current_command_a,
                dt_s * 1000.0f,
                static_cast<float>(now_ms - feedback.timestamp_ms),
            };
            vofa_send(&vofa, channels, 11);
        }
    }

    /* Unreachable: the loop above never exits. */
    return 0;
}
