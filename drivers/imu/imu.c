#include "drivers/imu/imu.h"

#include <math.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include "drivers/kalman_filter/kalman_filter.h"
#include "drivers/pid/pid.h"

#define DT_DRV_COMPAT skywalker_imu

// ─── 从 DTS 提取 imu_config（ROM） ───
// accel-dev / gyro-dev / heat-dev / filter-dev 为 phandle。
// estimator 为 string，通过 DT_INST_PROP 提取，用于选择解算实现。
#define IMU_CONFIG_DEFINE(inst)                                           \
    static const imu_config imu_config_##inst = {                         \
        .accel_dev  = DEVICE_DT_GET(DT_INST_PHANDLE(inst, accel_dev)),    \
        .gyro_dev   = DEVICE_DT_GET(DT_INST_PHANDLE(inst, gyro_dev)),     \
        .heat_dev   = DEVICE_DT_GET(DT_INST_PHANDLE(inst, heat_dev)),     \
        .filter_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, filter_dev)),   \
        .pid_dev    = DEVICE_DT_GET(DT_INST_PHANDLE(inst, pid_dev)),      \
        .estimator  = DT_INST_PROP(inst, estimator),                      \
    };

/**
 * @brief Zephyr 设备初始化函数
 *
 * 检查所有子设备（accel / gyro / heat / filter）是否就绪，并将 data 清零。
 *
 * @param dev Zephyr 设备指针
 * @return 0 表示成功，-ENODEV 表示子设备不可用
 */
static int skywalker_imu_init(const struct device *dev) {
    const imu_config *cfg = dev->config;

    if (!device_is_ready(cfg->accel_dev)) {
        return -ENODEV;
    }
    if (!device_is_ready(cfg->gyro_dev)) {
        return -ENODEV;
    }
    if (!device_is_ready(cfg->heat_dev)) {
        return -ENODEV;
    }
    if (!device_is_ready(cfg->filter_dev)) {
        return -ENODEV;
    }
    if (!device_is_ready(cfg->pid_dev)) {
        return -ENODEV;
    }

    // 调用滤波器自身的初始化（如 EKF 设定初始四元数）
    const struct imu_filter_api *api = imu_get_api(cfg->estimator);
    if (api != NULL) {
        api->init(cfg->filter_dev);
    }

    imu_data *data = dev->data;
    data->temp = 0.0f;
    for (int i = 0; i < 3; i++) {
        data->accel[i] = 0.0f;
        data->gyro[i]  = 0.0f;
        data->angle[i] = 0.0f;
    }
    return 0;
}

// ─── 为单个 DT 实例注册 Zephyr device ───
// data 大小固定，直接定义；config 由 IMU_CONFIG_DEFINE 生成。
#define IMU_INST(inst)                                                     \
    IMU_CONFIG_DEFINE(inst);                                               \
    static imu_data imu_data_##inst;                                       \
    DEVICE_DT_DEFINE(DT_DRV_INST(inst),                                    \
                     skywalker_imu_init,                                   \
                     NULL,                                                 \
                     &imu_data_##inst,                                     \
                     &imu_config_##inst,                                   \
                     POST_KERNEL,                                          \
                     50,                                                   \
                     NULL);

// ─── 展开所有 status = "okay" 的 DT 实例 ───
DT_INST_FOREACH_STATUS_OKAY(IMU_INST)

// ─── 传感器数据获取 ───
/**
 * @brief 从加速度计和陀螺仪获取原始数据
 *
 * 触发采样后读取 accel / gyro / temp 并填入 data 结构体。
 *
 * @param dev IMU 设备指针
 */
void imu_fetch(const struct device *dev) {
    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;

    struct sensor_value val[3];

    // ─── 加速度 ───
    sensor_sample_fetch(cfg->accel_dev);
    sensor_channel_get(cfg->accel_dev, SENSOR_CHAN_ACCEL_XYZ, val);
    data->accel[0] = sensor_value_to_float(&val[0]);
    data->accel[1] = sensor_value_to_float(&val[1]);
    data->accel[2] = sensor_value_to_float(&val[2]);

    // ─── 角速度 ───
    sensor_sample_fetch(cfg->gyro_dev);
    sensor_channel_get(cfg->gyro_dev, SENSOR_CHAN_GYRO_XYZ, val);
    data->gyro[0] = sensor_value_to_float(&val[0]);
    data->gyro[1] = sensor_value_to_float(&val[1]);
    data->gyro[2] = sensor_value_to_float(&val[2]);

    // ─── 温度 ───
    struct sensor_value temp;
    sensor_channel_get(cfg->accel_dev, SENSOR_CHAN_DIE_TEMP, &temp);
    data->temp = sensor_value_to_float(&temp);
}

// ─── 解算方法查找 ───
static const struct imu_filter_api *imu_get_api(const char *estimator) {
    if (strcmp(estimator, imu_estimator_ekf.name) == 0) return &imu_estimator_ekf.api;
    /* 后续新增算法在此添加 else if */
    return NULL;
}

// ─── 姿态解算调度 ───
/**
 * @brief 姿态解算入口
 *
 * 根据 cfg->estimator 通过 imu_get_api 查找并调用对应算法。
 * 新增加法只需添加 imu_estimator_xxx + imu_get_api 中一行。
 *
 * @param dev IMU 设备指针
 * @param dt  采样周期（秒）
 */
void imu_estimate(const struct device *dev, float dt) {
    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;
    const struct imu_filter_api *api = imu_get_api(cfg->estimator);

    if (api == NULL) return;

    api->predict(cfg->filter_dev, data->gyro, dt, data->angle);
    api->correct(cfg->filter_dev, data->accel);
    api->get_angle(cfg->filter_dev, data->angle);
}

// ─── 温度控制 ───
/**
 * @brief PID 温度控制
 *
 * 通过 pid_dev 计算 PWM 脉宽并写入 heat_dev，对标 mambo IMU 恒温方案。
 * 建议低频调用（~10 Hz），PID 参数在 DT overlay 中配置。
 *
 * @param dev         IMU 设备指针
 * @param target_temp 目标温度 (°C)
 * @param dt          距上次调用时间（秒）
 */
void imu_heat_control(const struct device *dev, float target_temp, float dt) {
    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;

    // PID 计算：setpoint=target, measurement=current，无前馈
    float output = pid_update(cfg->pid_dev->data, cfg->pid_dev->config,
                              target_temp, data->temp, dt, 0.0f);

    // 负值截断（加热器不能制冷）
    if (output < 0.0f) output = 0.0f;

    // PWM 输出，周期 = 1/128 s ≈ 7.8 ms
    uint32_t period = PWM_SEC(1U) / 128U;
    pwm_set(cfg->heat_dev, 0, period, (uint32_t)output, PWM_POLARITY_NORMAL);
}



////////////////////////////////////////////////////////////////////////////////
//  EKF 姿态解算（四元数扩展卡尔曼滤波）
//  对标 mambo IMU_QuaternionEKF，包含零偏估计、LPF、卡方检验、自适应增益、Yaw 圈数
////////////////////////////////////////////////////////////////////////////////

// EKF 持久状态
static struct {
    float GyroBias[3];         // 陀螺零偏 (rad/s)
    float AccelFiltered[3];    // 加速度低通滤波值
    float ChiSquare;           // 卡方检验值
    float AdaptiveGainScale;   // 自适应增益
    float YawTotal;            // 连续 yaw（跨圈累计）
    float YawPrev;             // 上一拍 yaw
    int16_t YawRoundCount;     // 跨圈计数
    uint64_t UpdateCount;      // 累计调用次数
    uint8_t ConvergeFlag;      // 收敛标志
    uint64_t ErrorCount;       // 连续异常计数
} ekf;

#define EKF_Q1            10.0f // 四元数过程噪声系数
#define EKF_R             1e6f // 加速度观测噪声
#define EKF_CHI_THRESHOLD 1e-8f // 卡方检验阈值

/** @brief 快速 1/sqrt(x) */
static float inv_sqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 初始化：X = [1,0,0,0]，清零 ekf 状态
 */
static void imu_ekf_init(const struct device *dev) {
    KalmanFilter *kf = (KalmanFilter *)dev->data;

    kf->X.pData[0] = 1.0f;
    kf->X.pData[1] = 0.0f;
    kf->X.pData[2] = 0.0f;
    kf->X.pData[3] = 0.0f;

    memset(&ekf, 0, sizeof(ekf));
    ekf.AdaptiveGainScale = 1.0f;
}

/**
 * @brief 预测：零偏估计 + 四元数运动学 F + 协方差传播
 */
static void imu_ekf_predict(const struct device *dev, const float gyro[3], float dt, float angle[3]) {
    KalmanFilter *kf = (KalmanFilter *)dev->data;

    // ─── 1. 陀螺零偏在线估计（静止时 LPF 累积） ───
    float gyro_norm = inv_sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    if (gyro_norm > 0.0f) gyro_norm = 1.0f / gyro_norm;

    float acc_norm_val;
    arm_sqrt_f32(ekf.AccelFiltered[0] * ekf.AccelFiltered[0] +
                 ekf.AccelFiltered[1] * ekf.AccelFiltered[1] +
                 ekf.AccelFiltered[2] * ekf.AccelFiltered[2], &acc_norm_val);
    if (gyro_norm < 0.2f && fabsf(acc_norm_val - 9.8f) < 0.35f) {
        if (ekf.UpdateCount == 0) {
            ekf.GyroBias[0] = gyro[0]; ekf.GyroBias[1] = gyro[1]; ekf.GyroBias[2] = gyro[2];
        }
        ekf.GyroBias[0] = ekf.GyroBias[0] * 0.9995f + gyro[0] * 0.0005f;
        ekf.GyroBias[1] = ekf.GyroBias[1] * 0.9995f + gyro[1] * 0.0005f;
        ekf.GyroBias[2] = ekf.GyroBias[2] * 0.9995f + gyro[2] * 0.0005f;
    }

    // ─── 2. 减去零偏 ───
    float gx = gyro[0] - ekf.GyroBias[0];
    float gy = gyro[1] - ekf.GyroBias[1];
    float gz = gyro[2] - ekf.GyroBias[2];

    // ─── 3. F = I + 0.5·Ω·dt ───
    float hwx = 0.5f * gx * dt, hwy = 0.5f * gy * dt, hwz = 0.5f * gz * dt;
    kf->F.pData[0]  = 1.0f;  kf->F.pData[1]  = -hwx; kf->F.pData[2]  = -hwy; kf->F.pData[3]  = -hwz;
    kf->F.pData[4]  = hwx;   kf->F.pData[5]  = 1.0f;  kf->F.pData[6]  = hwz;  kf->F.pData[7]  = -hwy;
    kf->F.pData[8]  = hwy;   kf->F.pData[9]  = -hwz;  kf->F.pData[10] = 1.0f;  kf->F.pData[11] = hwx;
    kf->F.pData[12] = hwz;   kf->F.pData[13] = hwy;   kf->F.pData[14] = -hwx;  kf->F.pData[15] = 1.0f;

    // ─── 4. Q = EKF_Q1·dt·I ───
    for (int i = 0; i < 16; i++) kf->Q.pData[i] = 0.0f;
    float qv = EKF_Q1 * dt;
    kf->Q.pData[0] = qv; kf->Q.pData[5] = qv; kf->Q.pData[10] = qv; kf->Q.pData[15] = qv;

    KalmanFilter_Predict(kf);

    float n = inv_sqrt(kf->X.pData[0] * kf->X.pData[0] + kf->X.pData[1] * kf->X.pData[1] +
                       kf->X.pData[2] * kf->X.pData[2] + kf->X.pData[3] * kf->X.pData[3]);
    if (n > 0.0f) for (int i = 0; i < 4; i++) kf->X.pData[i] *= n;
}

/**
 * @brief 修正：LPF + 卡方检验 + 自适应增益 + 卡尔曼更新
 */
static void imu_ekf_correct(const struct device *dev, const float accel[3]) {
    KalmanFilter *kf = (KalmanFilter *)dev->data;

    // ─── 1. 加速度低通滤波 ───
    if (ekf.UpdateCount == 0) {
        ekf.AccelFiltered[0] = accel[0];
        ekf.AccelFiltered[1] = accel[1];
        ekf.AccelFiltered[2] = accel[2];
    }
    float lpf = 0.05f;  // 截止频率 ≈ 3 Hz
    ekf.AccelFiltered[0] = ekf.AccelFiltered[0] * (1.0f - lpf) + accel[0] * lpf;
    ekf.AccelFiltered[1] = ekf.AccelFiltered[1] * (1.0f - lpf) + accel[1] * lpf;
    ekf.AccelFiltered[2] = ekf.AccelFiltered[2] * (1.0f - lpf) + accel[2] * lpf;

    // ─── 2. z = AccelFiltered / |AccelFiltered| ───
    float acc_norm;
    arm_sqrt_f32(ekf.AccelFiltered[0] * ekf.AccelFiltered[0] +
                 ekf.AccelFiltered[1] * ekf.AccelFiltered[1] +
                 ekf.AccelFiltered[2] * ekf.AccelFiltered[2], &acc_norm);
    if (acc_norm < 0.01f) return;
    float z_buf[3] = {ekf.AccelFiltered[0] / acc_norm,
                      ekf.AccelFiltered[1] / acc_norm,
                      ekf.AccelFiltered[2] / acc_norm};

    // ─── 3. 预测重力方向 h(q) 及 新息 ───
    float q0 = kf->X.pData[0], q1 = kf->X.pData[1];
    float q2 = kf->X.pData[2], q3 = kf->X.pData[3];
    float h0 = 2.0f * (q1 * q3 - q0 * q2);
    float h1 = 2.0f * (q0 * q1 + q2 * q3);
    float h2 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    float y0 = z_buf[0] - h0, y1 = z_buf[1] - h1, y2 = z_buf[2] - h2;
    ekf.ChiSquare = y0 * y0 + y1 * y1 + y2 * y2;

    // ─── 4. 卡方检验 ───
    float acc_mag;
    arm_sqrt_f32(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2], &acc_mag);
    int stable = (fabsf(acc_mag - 9.8f) < 0.25f);

    if (ekf.ChiSquare < 0.5f * EKF_CHI_THRESHOLD) ekf.ConvergeFlag = 1;

    if (ekf.ChiSquare > EKF_CHI_THRESHOLD && ekf.ConvergeFlag) {
        if (stable) ekf.ErrorCount++;
        else        ekf.ErrorCount = 0;
        if (ekf.ErrorCount > 50) {
            ekf.ConvergeFlag = 0;
        } else {
            return;  // 残差异常，跳过修正
        }
    } else {
        // ─── 5. 自适应增益 ───
        if (ekf.ChiSquare > 0.1f * EKF_CHI_THRESHOLD && ekf.ConvergeFlag)
            ekf.AdaptiveGainScale = (EKF_CHI_THRESHOLD - ekf.ChiSquare) / (0.9f * EKF_CHI_THRESHOLD);
        else
            ekf.AdaptiveGainScale = 1.0f;
        ekf.ErrorCount = 0;
    }

    // ─── 6. 自适应 R（等价于缩放 K）───
    float r_adj = EKF_R / ekf.AdaptiveGainScale;
    for (int i = 0; i < 9; i++) kf->R.pData[i] = 0.0f;
    kf->R.pData[0] = r_adj; kf->R.pData[4] = r_adj; kf->R.pData[8] = r_adj;

    // ─── 7. H = ∂h/∂q ───
    memset(kf->H.pData, 0, 12 * sizeof(float));
    kf->H.pData[0] = -2.0f * q2;  kf->H.pData[1] =  2.0f * q3;  kf->H.pData[2] = -2.0f * q0;  kf->H.pData[3] = 2.0f * q1;
    kf->H.pData[4] =  2.0f * q1;  kf->H.pData[5] =  2.0f * q0;  kf->H.pData[6] =  2.0f * q3;  kf->H.pData[7] = 2.0f * q2;
    kf->H.pData[8] =  2.0f * q0;  kf->H.pData[9] = -2.0f * q1;  kf->H.pData[10]= -2.0f * q2;  kf->H.pData[11]= 2.0f * q3;

    Matrix z;
    Matrix_Init(&z, 3, 1, z_buf);
    KalmanFilter_Update(kf, &z);

    float n = inv_sqrt(kf->X.pData[0] * kf->X.pData[0] + kf->X.pData[1] * kf->X.pData[1] +
                       kf->X.pData[2] * kf->X.pData[2] + kf->X.pData[3] * kf->X.pData[3]);
    if (n > 0.0f) for (int i = 0; i < 4; i++) kf->X.pData[i] *= n;

    ekf.UpdateCount++;
}

/**
 * @brief 四元数 → Euler (rad) + Yaw 圈数累计
 */
static void imu_ekf_get_angle(const struct device *dev, float angle[3]) {
    KalmanFilter *kf = (KalmanFilter *)dev->data;
    float q0 = kf->X.pData[0], q1 = kf->X.pData[1];
    float q2 = kf->X.pData[2], q3 = kf->X.pData[3];

    float roll, roll_tmp, pitch, yaw;
    float asin_arg = -2.0f * (q1 * q3 - q0 * q2);
    arm_sqrt_f32(1.0f - asin_arg * asin_arg, &roll_tmp);
    arm_atan2_f32(asin_arg, roll_tmp, &roll);
    arm_atan2_f32(2.0f * (q0 * q1 + q2 * q3), 2.0f * (q0 * q0 + q3 * q3) - 1.0f, &pitch);
    arm_atan2_f32(2.0f * (q0 * q3 + q1 * q2), 2.0f * (q0 * q0 + q1 * q1) - 1.0f, &yaw);

    // Yaw 跨圈累计
    if (yaw - ekf.YawPrev > M_PI)       ekf.YawRoundCount--;
    else if (yaw - ekf.YawPrev < -M_PI) ekf.YawRoundCount++;
    ekf.YawTotal = 2.0f * M_PI * ekf.YawRoundCount + yaw;
    ekf.YawPrev  = yaw;

    angle[0] = roll;
    angle[1] = pitch;
    angle[2] = yaw;
}

// ─── 解算方法注册 ───
// 每种算法声明一个 imu_estimator，命名规则：imu_estimator_<算法名>。
// 新增算法：实现 init/predict/correct/get_angle → 声明 imu_estimator_xxx → imu_get_api 加一行。
static const imu_estimator imu_estimator_ekf = {
    .name = "ekf",
    .api  = {
        .init      = imu_ekf_init,
        .predict   = imu_ekf_predict,
        .correct   = imu_ekf_correct,
        .get_angle = imu_ekf_get_angle,
    },
};

////////////////////////////////////////////////////////////////////////////////
