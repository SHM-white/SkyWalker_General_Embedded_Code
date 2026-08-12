#ifndef IMU_H
#define IMU_H

#include <zephyr/device.h>

/**
 * @brief IMU 解算滤波器通用 API
 *
 * 每种滤波器实现此接口后封装为 imu_estimator，
 * IMU 驱动通过 imu_get_api 按 estimator 字符串查找并调用。
 * 替换滤波器只需改 DT 中 estimator 和 filter-dev，无需改 IMU 代码。
 */
struct imu_filter_api {
    /** @brief 初始化滤波器状态 */
    void (*init)     (const struct device *dev);
    /** @brief 预测（陀螺仪积分 + 协方差传播） */
    void (*predict)  (const struct device *dev, const float gyro[3], float dt, float angle[3]);
    /** @brief 修正（加速度计重力观测更新） */
    void (*correct)  (const struct device *dev, const float accel[3]);
    /** @brief 读取姿态角 (rad), [roll, pitch, yaw] */
    void (*get_angle)(const struct device *dev, float angle[3]);
};

/** @brief 解算实现条目：名称 + API */
typedef struct {
    const char *name;
    const struct imu_filter_api api;
} imu_estimator;

/** @brief IMU 配置（ROM，由 DT 填充） */
typedef struct {
    const struct device *accel_dev;  /* 加速度计设备 */
    const struct device *gyro_dev;   /* 陀螺仪设备 */
    const struct device *heat_dev;   /* PWM 加热设备 */
    const struct device *filter_dev; /* 解算滤波器设备 */
    const struct device *pid_dev;     /* 温度 PID 设备 */
    const char *estimator;           /* 解算方法，如 "ekf" */
} imu_config;

/** @brief IMU 运行时数据 */
typedef struct {
    float accel[3];  /* 加速度 (m/s²), x/y/z */
    float gyro[3];   /* 角速度   (rad/s), x/y/z */
    float temp;      /* 温度     (°C) */
    float angle[3];  /* 姿态角   (rad), roll/pitch/yaw */
} imu_data;

/**
 * @brief 从传感器读取原始数据并填入 data
 * @param dev IMU 设备指针
 */
void imu_fetch(const struct device *dev);

/**
 * @brief 姿态解算
 * @param dev IMU 设备指针
 * @param dt  采样周期（秒）
 */
void imu_estimate(const struct device *dev, float dt);

/**
 * @brief PID 温度控制
 *
 * 低频调用（~10 Hz），通过 pid_dev 计算 PWM 脉宽并写入 heat_dev。
 *
 * @param dev         IMU 设备指针
 * @param target_temp 目标温度 (°C)
 * @param dt          距上次调用时间（秒）
 */
void imu_heat_control(const struct device *dev, float target_temp, float dt);

#endif // IMU_H