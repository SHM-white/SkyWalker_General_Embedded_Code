#include "lib/matrix/matrix.h" // 自己的 API 声明

// Zephyr 头文件
#include <zephyr/kernel.h>            // K_MUTEX_DEFINE, k_mutex_lock
#include <zephyr/device.h>            // device_is_ready
#include <zephyr/drivers/flash.h>     // flash_get_page_info_by_offs
#include <zephyr/storage/flash_map.h> // FIXED_PARTITION_* 从 DTS 读取分区信息
#include <zephyr/kvss/zms.h>          // zms_mount, zms_write, zms_read...
#include <zephyr/sys/util.h>          // is_power_of_two
#include <string.h>                   // memcpy

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(matrix_storage, LOG_LEVEL_INF);

// ─── ZMS 初始化（懒加载：第一次 save/read 时自动调用） ───

/*
 * 从 DTS 的 storage_partition { reg = <0xC0000 0x40000>; } 自动提取：
 *   DEVICE → flash 控制器设备（&flash0）
 *   OFFSET → 分区起始偏移（0x000C0000）
 *   SIZE   → 分区大小（256KB）
 */
#define MATRIX_STORAGE_PARTITION storage_partition
#define ZMS_PARTITION_DEVICE PARTITION_DEVICE(MATRIX_STORAGE_PARTITION)
#define ZMS_PARTITION_OFFSET PARTITION_OFFSET(MATRIX_STORAGE_PARTITION)
#define ZMS_PARTITION_SIZE PARTITION_SIZE(MATRIX_STORAGE_PARTITION)

static struct zms_fs fs;         // ZMS 文件系统实例（全局唯一）
static bool zms_ready;           // 是否已 mount（懒加载标记）
static K_MUTEX_DEFINE(zms_lock); // 互斥锁：防止多线程同时读写 flash

/**
 * @brief 初始化 ZMS 文件系统（懒加载）
 *
 * 获取 flash 页大小并计算扇区参数后挂载 ZMS。
 * ZMS 至少需要 2 个扇区：一个活跃，一个做垃圾回收（GC）。
 *
 * @return 0 成功，负数 errno 失败
 */
static int zms_init(void)
{
    if (zms_ready)
        return 0; // 已初始化，跳过

    // 获取 flash 设备指针
    fs.flash_device = ZMS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device))
    {
        LOG_ERR("Flash device not ready");
        return -ENODEV;
    }
    fs.offset = ZMS_PARTITION_OFFSET;

    // 读取 flash 页大小（擦除单位）
    struct flash_pages_info info;
    int rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc)
        return rc;

    // ZMS 要求扇区大小必须是 2 的幂
    if (!is_power_of_two(info.size))
        return -EINVAL;

    // 一个 ZMS 扇区 = 一个 flash 页
    fs.sector_size = info.size;
    fs.sector_count = ZMS_PARTITION_SIZE / info.size;

    // ZMS 最少需要 2 个扇区
    if (fs.sector_count < 2)
        return -EINVAL;

    rc = zms_mount(&fs);
    if (rc)
        return rc;

    zms_ready = true;
    return 0;
}

/**
 * @brief 保存任意尺寸矩阵到 flash（按 ID 区分）
 * @param id 矩阵 ID
 * @param data 矩阵数据指针
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * @return 0 成功，其他值失败
 */
int matrix_storage_save(uint32_t id, const float *data, uint16_t rows, uint16_t cols)
{
    if (!data || rows == 0 || cols == 0)
    {
        return -EINVAL; // 无效参数
    }

    uint16_t data_bytes = rows * cols * sizeof(float); // 矩阵数据大小
    uint16_t total_size = 4 + data_bytes;              // 头 4 字节（行，列各 2 字节） + 数据
    uint8_t buf[total_size];                           // 可变长度数组，栈上分配

    memcpy(buf + 0, &rows, 2);         // 写入 rows（小端序，STM32 原生就是）
    memcpy(buf + 2, &cols, 2);         // 写入 cols
    memcpy(buf + 4, data, data_bytes); // 写入矩阵浮点数据

    k_mutex_lock(&zms_lock, K_FOREVER); // 拿锁

    int rc = zms_init(); // 懒加载，第一次调用时 mount
    if (rc)
    {
        k_mutex_unlock(&zms_lock); // 初始化失败，放锁
        return rc;
    }

    rc = zms_write(&fs, id, buf, total_size); // 写 flash
    if (rc < 0)
    {
        LOG_ERR("zms_write failed: %d", rc); // 写入失败
    }
    else
    {
        rc = 0; // 写入成功
    }

    k_mutex_unlock(&zms_lock); // 放锁
    return rc;
}

/**
 * @brief 查询 flash 中矩阵的尺寸（只读 4 字节头）
 * @param id 矩阵 ID
 * @param rows 输出：行数
 * @param cols 输出：列数
 * @return 0 成功，其他值失败
 */
int matrix_storage_get_size(uint32_t id, uint16_t *rows, uint16_t *cols)
{
    if (!rows || !cols)
        return -EINVAL;

    uint8_t header[4];

    k_mutex_lock(&zms_lock, K_FOREVER);

    int rc = zms_init();
    if (rc)
    {
        k_mutex_unlock(&zms_lock);
        return rc;
    }

    ssize_t n = zms_read(&fs, id, header, sizeof(header));
    if (n < 0)
    {
        k_mutex_unlock(&zms_lock);
        return (int)n;
    }
    if (n < 4)
    {
        k_mutex_unlock(&zms_lock);
        return -ENODATA;
    }

    memcpy(rows, header + 0, 2);
    memcpy(cols, header + 2, 2);

    k_mutex_unlock(&zms_lock);
    return 0;
}

/**
 * @brief 从 flash 读取矩阵到 Matrix 结构体
 * @param id 矩阵 ID
 * @param mat 目标矩阵（调用者需提前分配 pData 并设置 numRows/numCols）
 * @return 0 成功，其他值失败
 */
int matrix_storage_read(uint32_t id, Matrix *mat)
{
    if (!mat || !mat->pData || mat->numRows == 0 || mat->numCols == 0)
        return -EINVAL;

    uint16_t stored_rows, stored_cols;

    k_mutex_lock(&zms_lock, K_FOREVER);

    int rc = zms_init();
    if (rc)
    {
        k_mutex_unlock(&zms_lock);
        return rc;
    }

    // 先读 4 字节头，获取存储的尺寸
    uint8_t header[4];
    ssize_t n = zms_read(&fs, id, header, sizeof(header));
    if (n < 0)
    {
        k_mutex_unlock(&zms_lock);
        return (int)n;
    }
    if (n < 4)
    {
        k_mutex_unlock(&zms_lock);
        return -ENODATA;
    }

    memcpy(&stored_rows, header + 0, 2);
    memcpy(&stored_cols, header + 2, 2);

    // 校验尺寸是否匹配
    if (stored_rows != mat->numRows || stored_cols != mat->numCols)
    {
        k_mutex_unlock(&zms_lock);
        return -EINVAL;
    }

    // 读完整数据
    uint16_t data_bytes = stored_rows * stored_cols * sizeof(float);
    uint16_t total = 4 + data_bytes;
    uint8_t buf[total];

    n = zms_read(&fs, id, buf, total);
    if (n < 0)
    {
        k_mutex_unlock(&zms_lock);
        return (int)n;
    }
    if (n < total)
    {
        k_mutex_unlock(&zms_lock);
        return -ENODATA;
    }

    memcpy(mat->pData, buf + 4, data_bytes);

    k_mutex_unlock(&zms_lock);
    return 0;
}

/**
 * @brief 检查指定 ID 的矩阵是否存在于 flash
 * @param id 矩阵 ID
 * @return true 存在，false 不存在
 */
bool matrix_storage_exists(uint32_t id)
{
    k_mutex_lock(&zms_lock, K_FOREVER);

    int rc = zms_init();
    if (rc)
    {
        k_mutex_unlock(&zms_lock);
        return false;
    }

    ssize_t len = zms_get_data_length(&fs, id);

    k_mutex_unlock(&zms_lock);
    return (len > 0);
}

/**
 * @brief 从 flash 删除指定 ID 的矩阵
 * @param id 矩阵 ID
 * @return 0 成功，其他值失败
 */
int matrix_storage_delete(uint32_t id)
{
    k_mutex_lock(&zms_lock, K_FOREVER);

    int rc = zms_init();
    if (rc)
    {
        k_mutex_unlock(&zms_lock);
        return rc;
    }

    rc = zms_delete(&fs, id);

    k_mutex_unlock(&zms_lock);
    return rc;
}
