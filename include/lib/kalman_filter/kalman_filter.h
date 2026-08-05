#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "lib/matrix/matrix.h"               // Matrix_Init 宏 + Matrix 类型

/**
 * @brief 卡尔曼滤波器结构体
 *
 * 所有矩阵的 pData 由调用者分配，通过 KalmanFilter_Init 绑定。
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

/**
 * @brief 初始化卡尔曼滤波器，将所有矩阵绑定到调用者分配的 buffer
 *
 * @param kf     滤波器结构体指针
 * @param n      状态向量维度
 * @param m      观测向量维度（无观测时传 0）
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
                       float *K_buf);

void KalmanFilter_Predict(KalmanFilter *kf);
void KalmanFilter_Update(KalmanFilter *kf, const Matrix *z);

#endif // KALMAN_FILTER_H