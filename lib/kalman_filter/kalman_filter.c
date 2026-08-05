#include "lib/kalman_filter/kalman_filter.h"


/**
 * @brief 初始化卡尔曼滤波器，将所有矩阵绑定到调用者分配的 buffer
 *
 * 调用者按尺寸分配 float 数组，init 只绑定，不分配内存。
 * m=0 时 H/R/K 的 pData 传 NULL，跳过更新步骤。
 *
 * @param kf     滤波器结构体指针
 * @param n      状态向量维度
 * @param m      观测向量维度
 * @param F_buf  F 矩阵 buffer，长度 n*n
 * @param H_buf  H 矩阵 buffer，长度 m*n
 * @param R_buf  R 矩阵 buffer，长度 m*m
 * @param X_buf  X 矩阵 buffer，长度 n
 * @param P_buf  P 矩阵 buffer，长度 n*n
 * @param Q_buf  Q 矩阵 buffer，长度 n*n
 * @param K_buf  K 矩阵 buffer，长度 n*m
 */
void KalmanFilter_Init(KalmanFilter *kf, uint16_t n, uint16_t m,
                       float *F_buf,
                       float *H_buf, float *R_buf,
                       float *X_buf, float *P_buf, float *Q_buf,
                       float *K_buf)
{
    // ─── 预测参数 ───
    // F: 状态转移矩阵 (n×n)，描述系统自身动力学
    Matrix_Init(&kf->F, n, n, F_buf);

    // ─── 更新参数 ───
    // H: 观测矩阵 (m×n)，从状态空间中挑出可观测分量
    Matrix_Init(&kf->H, m, n, H_buf);

    // R: 测量噪声协方差 (m×m)，表征传感器不确定度
    Matrix_Init(&kf->R, m, m, R_buf);

    // ─── 状态变量 ───
    // X: 状态估计向量 (n×1)，滤波器的最终输出
    Matrix_Init(&kf->X, n, 1, X_buf);

    // P: 误差协方差矩阵 (n×n)，当前估计的不确定度
    Matrix_Init(&kf->P, n, n, P_buf);

    // Q: 过程噪声协方差 (n×n)，系统模型的不确定度
    Matrix_Init(&kf->Q, n, n, Q_buf);

    // ─── 卡尔曼增益（内部计算） ───
    // K: 卡尔曼增益 (n×m)，update 中计算，决定观测对状态的修正权重
    Matrix_Init(&kf->K, n, m, K_buf);
}

/**
 * @brief 预测步骤：x = F·x,  P = F·P·Fᵀ + Q
 * @param kf 滤波器结构体指针
 */
void KalmanFilter_Predict(KalmanFilter *kf)
{
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
void KalmanFilter_Update(KalmanFilter *kf, const Matrix *z)
{
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
