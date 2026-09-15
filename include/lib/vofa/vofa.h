#ifndef VOFA_H
#define VOFA_H

#include <zephyr/device.h>
#include <zephyr/spinlock.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 收到完整 key=value 行时调用，key 仅在回调期间有效。
 * 在 UART 驱动回调中执行（USART 可能是 ISR，CDC ACM 是工作队列），不能阻塞。
 */
typedef void (*vofa_cmd_handler)(const char *key, float val);

#define VOFA_MAX_FLOATS 16
#define VOFA_TX_QUEUE_DEPTH 4
#define VOFA_FRAME_SIZE ((VOFA_MAX_FLOATS + 1) * sizeof(float))

/** 初始化后不得复制、重新初始化或移动，生命周期须覆盖 UART 回调。 */
typedef struct {
    const struct device *uart;

    /* Private TX state: complete frames, owned by this instance. */
    struct k_spinlock tx_lock;
    uint8_t tx_frames[VOFA_TX_QUEUE_DEPTH][VOFA_FRAME_SIZE];
    uint8_t tx_lengths[VOFA_TX_QUEUE_DEPTH];
    uint8_t tx_head;
    uint8_t tx_count;
    uint8_t tx_offset;
    int tx_error;

    /* Private RX state; caller supplies storage through vofa_set_handler(). */
    uint8_t *rx_buf;
    size_t rx_buf_size;
    size_t rx_length;
    bool rx_discard;
    vofa_cmd_handler on_cmd;
} Vofa;

/**
 * 绑定已初始化、支持 IRQ/FIFO API 的 USART 或 USB CDC ACM UART。
 * USB 栈由应用/Zephyr 初始化；本函数不等待主机连接。
 * 从线程中调用一次，设备须独占，不能与 Console/Shell/其他 UART 回调共用。
 * 返回 0、-EINVAL、-ENODEV 或 UART 回调注册错误。失败后不能发送。
 */
int vofa_init(Vofa *vofa, const struct device *uart);

/**
 * 非阻塞复制完整 JustFloat 帧到队列，可在线程或普通 ISR 中调用。
 * num 为 1..16。返回 0 仅表示入队成功，调用方可以立即复用 data。
 * 队列满返回 -ENOBUFS（整帧拒绝）；其他错误为 -EINVAL、-ENODEV 或驱动错误。
 * 不保证主机接收，不等待 USB/DTR；多个生产者可共享同一实例。
 */
int vofa_send(Vofa *vofa, const float *data, uint8_t num);

/**
 * 从线程中设置一次接收处理器，并自动启用 IRQ 接收，无需 uart_rx_enable()。
 * rx_buf 须至少 2 字节且在设备使用期间保持有效，存储跨回调的命令行。
 * 超长/包含 NUL 的行丢弃到下一个换行；支持 LF 与 CRLF。
 * 返回 0、-EINVAL、-ENODEV 或 -EALREADY。不支持并发配置或运行时替换。
 */
int vofa_set_handler(Vofa *vofa, uint8_t *rx_buf, size_t rx_buf_size, vofa_cmd_handler on_cmd);

#ifdef __cplusplus
}
#endif

#endif // VOFA_H
