#include "drivers/pid/pid.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define DT_DRV_COMPAT skywalker_pid

/**
 * @brief 简易 atof，解析 DTS 字符串为 float
 *
 * 不支持科学计数法，覆盖 PID 参数的字符串格式已足够。
 */
static float parse_float(const char *s) {
    float result = 0.0f;
    float sign   = 1.0f;

    if (*s == '-') {
        sign = -1.0f;
        s++;
    }

    // ─── 整数部分 ───
    while (*s >= '0' && *s <= '9') {
        result = result * 10.0f + (*s - '0');
        s++;
    }

    // ─── 小数部分 ───
    if (*s == '.') {
        s++;
        float frac = 0.1f;
        while (*s >= '0' && *s <= '9') {
            result += (*s - '0') * frac;
            frac *= 0.1f;
            s++;
        }
    }

    return result * sign;
}

// ─── 从 DTS 提取 pid_config（ROM） ───
// DTS 不支持 float，所有属性为 string 类型，用 parse_float 转换。
// DTS 属性名 k-p / i-max 等在宏中为 k_p / i_max（横线→下划线）。
#define PID_CONFIG_DEFINE(inst)                                      \
    static const pid_config pid_config_##inst = {                    \
        .Kp        = parse_float(DT_STRING_UNQUOTED(                 \
                          DT_DRV_INST(inst), k_p)),                  \
        .Ki        = parse_float(DT_STRING_UNQUOTED(                 \
                          DT_DRV_INST(inst), k_i)),                  \
        .Kd        = parse_float(DT_STRING_UNQUOTED(                 \
                          DT_DRV_INST(inst), k_d)),                  \
        .max_i_out = parse_float(DT_STRING_UNQUOTED(                 \
                          DT_DRV_INST(inst), i_max)),                \
        .max_out   = parse_float(DT_STRING_UNQUOTED(                 \
                          DT_DRV_INST(inst), out_max)),              \
        .deadband  = parse_float(DT_STRING_UNQUOTED(                 \
                          DT_DRV_INST(inst), deadband)),             \
    };

/**
 * @brief Zephyr 设备初始化函数
 *
 * 系统启动时由 Zephyr 自动调用，将 data 区域清零。
 *
 * @param dev Zephyr 设备指针
 * @return 0 表示成功
 */
static int skywalker_pid_init(const struct device *dev) {
    pid_data *data = dev->data;
    data->last_error = 0.0f;
    data->p_out      = 0.0f;
    data->i_out      = 0.0f;
    data->d_out      = 0.0f;
    data->output     = 0.0f;
    return 0;
}
// ─── 为单个 DT 实例注册 Zephyr device ───
#define PID_INST(inst)                                                       \
    PID_CONFIG_DEFINE(inst);                                                 \
    static pid_data pid_data_##inst;                                         \
    DEVICE_DT_DEFINE(DT_DRV_INST(inst),                                      \
                     skywalker_pid_init,                                     \
                     NULL,                                                   \
                     &pid_data_##inst,                                       \
                     &pid_config_##inst,                                     \
                     POST_KERNEL,                                            \
                     50,                                                     \
                     NULL);

DT_INST_FOREACH_STATUS_OKAY(PID_INST)

/**
 * @brief 初始化 PID 状态，将所有运行时变量清零
 *
 * @param data PID 状态结构体指针
 */
/**
 * @brief PID 增量式计算
 *
 * 死区判断后依次计算 P / I / D 三项：
 *   P_out = Kp × error
 *   I_out += Ki × error × dt → 钳位至 ±max_i_out
 *   D_out = Kd × (error - last_error) / dt
 *   output = P_out + I_out + D_out + feedforward → 钳位至 ±max_out
 *
 * i_out 同时承担积分累加器和 I 项输出两个角色。
 *
 * @param data        PID 状态（读写，last_error 在函数内更新）
 * @param config      PID 配置参数（只读）
 * @param setpoint    目标值
 * @param measurement 测量值
 * @param dt          采样周期（秒），不得为 0
 * @param feedforward 前馈量（由调用者根据工况计算）
 * @return 控制输出
 */
float pid_update(pid_data *data,
                 const pid_config *config,
                 float setpoint, float measurement, float dt,
                 float feedforward) {
    // ─── 误差 ───
    float error = setpoint - measurement;

    // ─── 死区 ───
    if (error < config->deadband && error > -config->deadband) {
        data->last_error = error;
        data->p_out      = 0.0f;
        data->d_out      = 0.0f;
        data->output     = 0.0f;
        return 0.0f;
    }

    // ─── P ───
    data->p_out = config->Kp * error;

    // ─── I ───
    data->i_out += config->Ki * error * dt;
    if (data->i_out > config->max_i_out) {
        data->i_out = config->max_i_out;
    } else if (data->i_out < -config->max_i_out) {
        data->i_out = -config->max_i_out;
    }

    // ─── D ───
    data->d_out = config->Kd * (error - data->last_error) / dt;

    // ─── 更新上一拍误差 ───
    data->last_error = error;

    // ─── 总输出 ───
    data->output = data->p_out + data->i_out + data->d_out + feedforward;
    if (data->output > config->max_out) {
        data->output = config->max_out;
    } else if (data->output < -config->max_out) {
        data->output = -config->max_out;
    }

    return data->output;
}
