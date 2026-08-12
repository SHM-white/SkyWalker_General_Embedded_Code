#include "drivers/kalman_filter/kalman_filter.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define DT_DRV_COMPAT skywalker_kalman_filter

#define KALMAN_FILTER_CONFIG_DEFINE(inst)                                    \
    static const kalmanfilter_config kalmanfilter_config_##inst = {          \
        .state_dim   = DT_INST_PROP(inst, state_dim),                        \
        .measure_dim = DT_INST_PROP(inst, measure_dim),                      \
    }

#define KF_N(inst) DT_INST_PROP(inst, state_dim)
#define KF_M(inst) DT_INST_PROP(inst, measure_dim)

#define KALMAN_FILTER_DATA_DEFINE(inst)                                      \
    static struct {                                                          \
        KalmanFilter kf;                                                     \
        float F_buf[KF_N(inst) * KF_N(inst)];                                \
        float H_buf[KF_M(inst) * KF_N(inst)];                                \
        float R_buf[KF_M(inst) * KF_M(inst)];                                \
        float X_buf[KF_N(inst)];                                             \
        float P_buf[KF_N(inst) * KF_N(inst)];                                \
        float Q_buf[KF_N(inst) * KF_N(inst)];                                \
        float K_buf[KF_N(inst) * KF_M(inst)];                                \
    } kalman_filter_data_##inst

static int skywalker_kalman_filter_init(const struct device *dev) {
    const kalmanfilter_config *cfg = dev->config;
    KalmanFilter              *kf  = dev->data;
    uint16_t n = cfg->state_dim;
    uint16_t m = cfg->measure_dim;

    /* 指针偏移定位 data 内部各 buffer */
    uint8_t *base  = (uint8_t *)dev->data;
    float   *F_buf = (float *)(base + sizeof(KalmanFilter));
    float   *H_buf = F_buf + n * n;
    float   *R_buf = H_buf + m * n;
    float   *X_buf = R_buf + m * m;
    float   *P_buf = X_buf + n;
    float   *Q_buf = P_buf + n * n;
    float   *K_buf = Q_buf + n * n;

    /* 绑定矩阵 */
    Matrix_Init(&kf->F, n, n, F_buf);
    Matrix_Init(&kf->H, m, n, H_buf);
    Matrix_Init(&kf->R, m, m, R_buf);
    Matrix_Init(&kf->X, n, 1, X_buf);
    Matrix_Init(&kf->P, n, n, P_buf);
    Matrix_Init(&kf->Q, n, n, Q_buf);
    Matrix_Init(&kf->K, n, m, K_buf);

    /* 设置初始值 */
    Matrix_SetDiag(&kf->F, 1.0f);        /* F = I */
    Matrix_SetDiag(&kf->P, 1000.0f);     /* 高初始不确定度 */
    Matrix_SetDiag(&kf->Q, 0.001f);      /* 过程噪声 */
    Matrix_SetDiag(&kf->R, 1.0f);        /* 测量噪声 */
    Matrix_Zero(&kf->H);
    for (uint16_t i = 0; i < m && i < n; i++)
        kf->H.pData[i * n + i] = 1.0f;   /* H = I 的前 m 行 */
    Matrix_Zero(&kf->X);                 /* X = 0 */

    return 0;
}

#define KALMAN_FILTER_INST(inst)                                             \
    KALMAN_FILTER_CONFIG_DEFINE(inst);                                       \
    KALMAN_FILTER_DATA_DEFINE(inst);                                         \
    DEVICE_DT_DEFINE(DT_DRV_INST(inst),                                      \
                     skywalker_kalman_filter_init,                           \
                     NULL,                                                   \
                     &kalman_filter_data_##inst,                             \
                     &kalmanfilter_config_##inst,                            \
                     POST_KERNEL,                                            \
                     50,                                                     \
                     NULL);

DT_INST_FOREACH_STATUS_OKAY(KALMAN_FILTER_INST)

/**
 * @brief 预测步骤：x = F·x,  P = F·P·Fᵀ + Q
 * @param kf 滤波器结构体指针
 */
void KalmanFilter_Predict(KalmanFilter *kf) {
    uint16_t n = kf->F.numRows;

    // ─── ① x = F·x ───
    // 源和目标为同一矩阵，先算到 tmp 再拷贝回去
    float x_tmp_buf[n];
    Matrix x_tmp;
    Matrix_Init(&x_tmp, n, 1, x_tmp_buf);
    Matrix_Multiply(&kf->F, &kf->X, &x_tmp);
    memcpy(kf->X.pData, x_tmp_buf, n * sizeof(float));

    // ─── ② P = F·P·Fᵀ + Q ───
    // FP = F * P
    float FP_buf[n * n];
    Matrix FP;
    Matrix_Init(&FP, n, n, FP_buf);
    Matrix_Multiply(&kf->F, &kf->P, &FP);

    // FT = Fᵀ
    float FT_buf[n * n];
    Matrix FT;
    Matrix_Init(&FT, n, n, FT_buf);
    Matrix_Transpose(&kf->F, &FT);

    // FPF_T = FP * FT = F·P·Fᵀ
    float FPF_T_buf[n * n];
    Matrix FPF_T;
    Matrix_Init(&FPF_T, n, n, FPF_T_buf);
    Matrix_Multiply(&FP, &FT, &FPF_T);

    // P = FPF_T + Q  （直接写入 kf->P，旧 P 已用完）
    Matrix_Add(&FPF_T, &kf->Q, &kf->P);
}

/**
 * @brief 更新步骤：K = P·Hᵀ·(H·P·Hᵀ+R)⁻¹,  x = x+K·(z-H·x),  P = (I-K·H)·P
 * @param kf 滤波器结构体指针
 * @param z  观测向量 (m×1)
 */
void KalmanFilter_Correct(KalmanFilter *kf, const Matrix *z) {
    uint16_t n = kf->F.numRows;
    uint16_t m = kf->H.numRows;

    // ═══ ③ K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹ ═══

    // HT = Hᵀ (n×m)
    float HT_buf[n * m];
    Matrix HT;
    Matrix_Init(&HT, n, m, HT_buf);
    Matrix_Transpose(&kf->H, &HT);

    // P_HT = P * Hᵀ (n×m)
    float P_HT_buf[n * m];
    Matrix P_HT;
    Matrix_Init(&P_HT, n, m, P_HT_buf);
    Matrix_Multiply(&kf->P, &HT, &P_HT);

    // HP = H * P (m×n)
    float HP_buf[m * n];
    Matrix HP;
    Matrix_Init(&HP, m, n, HP_buf);
    Matrix_Multiply(&kf->H, &kf->P, &HP);

    // HPH_T = HP * Hᵀ = H·P·Hᵀ (m×m)
    float HPH_T_buf[m * m];
    Matrix HPH_T;
    Matrix_Init(&HPH_T, m, m, HPH_T_buf);
    Matrix_Multiply(&HP, &HT, &HPH_T);

    // S = HPH_T + R (m×m)
    float S_buf[m * m];
    Matrix S;
    Matrix_Init(&S, m, m, S_buf);
    Matrix_Add(&HPH_T, &kf->R, &S);

    // S_inv = S⁻¹ (m×m)
    float S_inv_buf[m * m];
    Matrix S_inv;
    Matrix_Init(&S_inv, m, m, S_inv_buf);
    Matrix_Inverse(&S, &S_inv);

    // K = P_HT * S_inv (n×m)，写入 kf->K
    Matrix_Multiply(&P_HT, &S_inv, &kf->K);

    // ═══ ④ x = x + K·(z - H·x) ═══

    // HX = H * x (m×1)
    float HX_buf[m];
    Matrix HX;
    Matrix_Init(&HX, m, 1, HX_buf);
    Matrix_Multiply(&kf->H, &kf->X, &HX);

    // y = z - HX (m×1)  新息 (innovation)
    float y_buf[m];
    Matrix y;
    Matrix_Init(&y, m, 1, y_buf);
    Matrix_Subtract(z, &HX, &y);

    // Ky = K * y (n×1)
    float Ky_buf[n];
    Matrix Ky;
    Matrix_Init(&Ky, n, 1, Ky_buf);
    Matrix_Multiply(&kf->K, &y, &Ky);

    // x = x + Ky  源和目标重叠，中转
    float X_new_buf[n];
    Matrix X_new;
    Matrix_Init(&X_new, n, 1, X_new_buf);
    Matrix_Add(&kf->X, &Ky, &X_new);
    memcpy(kf->X.pData, X_new_buf, n * sizeof(float));

    // ═══ ⑤ P = (I - K·H)·P ═══

    // KH = K * H (n×n)
    float KH_buf[n * n];
    Matrix KH;
    Matrix_Init(&KH, n, n, KH_buf);
    Matrix_Multiply(&kf->K, &kf->H, &KH);

    // I_KH = I - K·H  (手动构建单位阵减 KH)
    float I_KH_buf[n * n];
    Matrix I_KH;
    Matrix_Init(&I_KH, n, n, I_KH_buf);
    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            I_KH_buf[i * n + j] = -KH_buf[i * n + j];
        }
        I_KH_buf[i * n + i] += 1.0f;
    }

    // P = (I - KH) * P  源和目标重叠，中转
    float P_new_buf[n * n];
    Matrix P_new;
    Matrix_Init(&P_new, n, n, P_new_buf);
    Matrix_Multiply(&I_KH, &kf->P, &P_new);
    memcpy(kf->P.pData, P_new_buf, n * n * sizeof(float));
}
