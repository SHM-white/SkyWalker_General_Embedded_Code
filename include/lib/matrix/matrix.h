#ifndef MATRIX_H
#define MATRIX_H

#include "arm_math.h"

#define Matrix              arm_matrix_instance_f32
#define Matrix_Init         arm_mat_init_f32
#define Matrix_Add          arm_mat_add_f32
#define Matrix_Subtract     arm_mat_sub_f32
#define Matrix_Multiply     arm_mat_mult_f32
#define Matrix_Transpose    arm_mat_trans_f32
#define Matrix_Inverse      arm_mat_inverse_f32

// 工具函数：将矩阵所有元素置零
static inline void Matrix_Zero(Matrix *mat)
{
    for (uint16_t i = 0; i < mat->numRows; i++)
        for (uint16_t j = 0; j < mat->numCols; j++)
            mat->pData[i * mat->numCols + j] = 0.0f;
}

// 工具函数：将矩阵设为对角阵（非对角置零，对角设为 val）
static inline void Matrix_SetDiag(Matrix *mat, float val)
{
    Matrix_Zero(mat);
    uint16_t n = (mat->numRows < mat->numCols) ? mat->numRows : mat->numCols;
    for (uint16_t i = 0; i < n; i++)
        mat->pData[i * mat->numCols + i] = val;
}

// 保存任意尺寸矩阵到 flash（按 ID 区分）
int matrix_storage_save(uint32_t id, const float *data, uint16_t rows, uint16_t cols);

// 查询 flash 中矩阵的尺寸（只读 4 字节头，不读数据）
int matrix_storage_get_size(uint32_t id, uint16_t *rows, uint16_t *cols);

// 从 flash 读取矩阵到 Matrix 结构体（调用者需提前分配 pData 并初始化 numRows/numCols）
int matrix_storage_read(uint32_t id, Matrix *mat);

// 检查 ID 是否存在
bool matrix_storage_exists(uint32_t id);

// 删除指定 ID
int matrix_storage_delete(uint32_t id);

#endif