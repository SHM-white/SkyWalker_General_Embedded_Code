#ifndef SKYWALKER_CONTROL_KALMAN_FILTER_H
#define SKYWALKER_CONTROL_KALMAN_FILTER_H

#include <stddef.h>
#include <stdint.h>
#include <lib/matrix/matrix.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float *data;
    size_t capacity; /* Number of float elements, not bytes. */
} KalmanBuffer;

typedef struct {
    KalmanBuffer F, H, R, X, P, Q, K;
} KalmanBuffers;

/**
 * @brief 卡尔曼滤波器结构体
 *
 * 所有矩阵的 pData 由调用者分配，通过 KalmanFilter_Init 绑定。
 * 数据缓冲必须互不重叠，且覆盖实例的整个使用周期；同一实例串行使用。
 *
 * 矩阵尺寸一览（n=状态维度, m=观测维度）：
 *   F(n×n)  状态转移矩阵
 *   H(m×n)  观测矩阵
 *   R(m×m)  测量噪声协方差
 *   x(n×1)  状态估计向量（唯一输出）
 *   P(n×n)  误差协方差矩阵
 *   Q(n×n)  过程噪声协方差
 *   K(n×m)  卡尔曼增益（内部计算，只读）
 */
typedef struct {
    Matrix F;       // 状态转移矩阵
    Matrix H, R;    // 观测矩阵, 测量噪声协方差
    Matrix X, P, Q; // 状态估计, 误差协方差, 过程噪声协方差
    Matrix K;       // 卡尔曼增益
} KalmanFilter;

/**
 * @brief Bind caller-owned buffers and initialize a linear Kalman filter.
 *
 * Requires n=state_dim > 0 and m=measure_dim > 0. Capacities are:
 * F/P/Q: n*n, H/K: n*m, R: m*m, X: n float elements.
 * Defaults: F=I, H has min(n,m) identity rows, P=1000I, Q=0.001I,
 * R=I, X=0 and K=0. Configure the application model before computing.
 *
 * Allocates no memory and retains only the buffer pointers, not the descriptor.
 * Reinitialization overwrites model and state; stop using the instance first.
 * Call from the owning thread, never concurrently with compute or reads.
 *
 * @return 0 on success; -EINVAL for null pointers or zero dimensions;
 * -ENOSPC for insufficient capacity; -EOVERFLOW if a dimension product
 * exceeds the matrix indexing or byte-count range. Failure changes nothing.
 */
int KalmanFilter_Init(KalmanFilter *kf, uint16_t state_dim, uint16_t measure_dim, const KalmanBuffers *buffers);

/* Initialized instance, single owner thread; dimension-sized stack scratch.
 * These legacy void operations do not propagate CMSIS-DSP errors. */
void KalmanFilter_Predict(KalmanFilter *kf);
void KalmanFilter_Correct(KalmanFilter *kf, const Matrix *z);

#ifdef __cplusplus
}
#endif

#endif /* SKYWALKER_CONTROL_KALMAN_FILTER_H */
