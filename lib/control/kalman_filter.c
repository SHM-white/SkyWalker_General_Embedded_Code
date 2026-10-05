#include <control/kalman_filter.h>

#include <errno.h>
#include <limits.h>
#include <string.h>

int KalmanFilter_Init(KalmanFilter *kf, uint16_t state_dim, uint16_t measure_dim, const KalmanBuffers *buffers) {
    if (!kf || !buffers || !state_dim || !measure_dim)
        return -EINVAL;

    const size_t n = state_dim, m = measure_dim;
    if (n > SIZE_MAX / n || m > SIZE_MAX / m || n > SIZE_MAX / m)
        return -EOVERFLOW;
    const size_t nn = n * n, nm = n * m, mm = m * m;
    /* Matrix helpers use int-promoted indices; byte sizes must also fit. */
    if (nn > INT_MAX || nm > INT_MAX || mm > INT_MAX || nn > SIZE_MAX / sizeof(float) ||
        nm > SIZE_MAX / sizeof(float) || mm > SIZE_MAX / sizeof(float))
        return -EOVERFLOW;

    const KalmanBuffer storage[] = {buffers->F, buffers->H, buffers->R, buffers->X, buffers->P, buffers->Q, buffers->K};
    const size_t required[] = {nn, nm, mm, n, nn, nn, nm};
    for (size_t i = 0; i < sizeof(storage) / sizeof(storage[0]); ++i)
        if (!storage[i].data)
            return -EINVAL;
    for (size_t i = 0; i < sizeof(storage) / sizeof(storage[0]); ++i)
        if (storage[i].capacity < required[i])
            return -ENOSPC;

    Matrix_Init(&kf->F, state_dim, state_dim, storage[0].data);
    Matrix_Init(&kf->H, measure_dim, state_dim, storage[1].data);
    Matrix_Init(&kf->R, measure_dim, measure_dim, storage[2].data);
    Matrix_Init(&kf->X, state_dim, 1, storage[3].data);
    Matrix_Init(&kf->P, state_dim, state_dim, storage[4].data);
    Matrix_Init(&kf->Q, state_dim, state_dim, storage[5].data);
    Matrix_Init(&kf->K, state_dim, measure_dim, storage[6].data);

    Matrix_SetDiag(&kf->F, 1.0f);
    Matrix_SetDiag(&kf->P, 1000.0f);
    Matrix_SetDiag(&kf->Q, 0.001f);
    Matrix_SetDiag(&kf->R, 1.0f);
    Matrix_Zero(&kf->H);
    for (size_t i = 0; i < m && i < n; ++i)
        kf->H.pData[i * n + i] = 1.0f;
    Matrix_Zero(&kf->X);
    Matrix_Zero(&kf->K);
    return 0;
}

/**
 * @brief 预测步骤：x = F·x,  P = F·P·Fᵀ + Q
 * @param kf 滤波器结构体指针
 */
void KalmanFilter_Predict(KalmanFilter *kf) {
    size_t n = kf->F.numRows;

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
    size_t n = kf->F.numRows;
    size_t m = kf->H.numRows;

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
