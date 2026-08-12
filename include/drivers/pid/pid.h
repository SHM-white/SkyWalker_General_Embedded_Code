#ifndef PID_H
#define PID_H

/** @brief PID 配置参数（只读，ROM） */
typedef struct {
    float Kp, Ki, Kd;   // 增益
    float max_i_out;     // I 项输出限幅
    float max_out;       // 总输出限幅
    float deadband;      // 死区
} pid_config;

/** @brief PID 运行时状态（读写，RAM） */
typedef struct {
    float last_error;           // 上一拍误差（D 项用）
    float p_out, i_out, d_out;  // 各项输出
    float output;               // 最终输出
} pid_data;

/** @brief PID 计算，返回控制量 */
float pid_update(pid_data *data,
                 const pid_config *config,
                 float setpoint, float measurement, float dt,
                 float feedforward);

#endif // PID_H
