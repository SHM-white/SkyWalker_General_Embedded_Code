#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "lib/matrix/matrix.h"               // Matrix_Init 宏 + Matrix 类型

typedef struct {
    uint16_t state_dim;      // 状态向量维度 n
    uint16_t measure_dim; // 观测向量维度 m
} kalmanfilter_config;

/**
 * @brief 卡尔曼滤波器结构体
 *
 * 所有矩阵的 pData 由调用者分配，在设备初始化时绑定。
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
    Matrix  F;         // 状态转移矩阵
    Matrix  H, R;     // 观测矩阵, 测量噪声协方差
    Matrix  X, P, Q;  // 状态估计, 误差协方差, 过程噪声协方差
    Matrix  K;        // 卡尔曼增益
} KalmanFilter;

void KalmanFilter_Predict(KalmanFilter *kf);
void KalmanFilter_Correct(KalmanFilter *kf, const Matrix *z);

#endif // KALMAN_FILTER_H