#ifndef VOFA_H
#define VOFA_H

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <stddef.h>

/**
 * @brief 文本命令回调 — 每收到一行完整 key=value 命令时调用
 *
 * 如 "kp=1.5\n" → 调用 handler("kp", 1.5)。
 *
 * @param key 命令名（"=" 左侧）
 * @param val 数值（"=" 右侧，由 parse_float 转换）
 */
typedef void (*vofa_cmd_handler)(const char *key, float val);

/**
 * @brief VOFA 调试通信实例
 *
 * TX: VOFA+ JustFloat 协议（原始 float 小端 + 0x7F800000 帧尾）
 * RX: DMA + IDLE 检测，文本行命令
 */
typedef struct {
    const struct device *uart;  // 绑定的 UART 设备

    // ─── RX ───
    uint8_t          *rx_buf;       // DMA 缓冲区（调用者分配）
    size_t            rx_buf_size;  // 缓冲区大小
    vofa_cmd_handler  on_cmd;       // 收到完整行时的回调
} Vofa;

void vofa_init(Vofa *vofa, const struct device *uart);

void vofa_send(Vofa *vofa, const float *data, uint8_t num);

void vofa_set_handler(Vofa *vofa, uint8_t *rx_buf, size_t rx_buf_size,
                      vofa_cmd_handler on_cmd);

void vofa_uart_cb(const struct device *dev, struct uart_event *evt,
                  void *user_data);

#endif // VOFA_H
