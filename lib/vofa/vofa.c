#include "lib/vofa/vofa.h"

#define VOFA_MAX_FLOATS 16  /* 单帧最多发送的 float 个数 */


/**
 * @brief 简易 atof，解析 "1.5" / "-0.01" / "100" 格式的字符串为 float
 *
 * 不支持科学计数法，覆盖 PID 调参场景已足够。
 * 写入时才链接，不依赖 libc。
 */
static float parse_float(const char *s) {
    float result = 0.0f;
    float sign   = 1.0f;

    if (*s == '-') {
        sign = -1.0f;
        s++;
    }

    // ─── 整数部分 ───
    while (*s >= '0' && *s <= '9') {
        result = result * 10.0f + (*s - '0');
        s++;
    }

    // ─── 小数部分 ───
    if (*s == '.') {
        s++;
        float frac = 0.1f;
        while (*s >= '0' && *s <= '9') {
            result += (*s - '0') * frac;
            frac *= 0.1f;
            s++;
        }
    }

    return result * sign;
}

/**
 * @brief 解析 "key=value" 行并回调 on_cmd
 *
 * 原地将 '=' 替换为 '\0'，左侧作为 key、右侧转为 float。
 */
static void process_line(Vofa *vofa, char *line, size_t len) {
    // ─── 查找 '=' ───
    char *eq = NULL;
    for (size_t i = 0; i < len; i++) {
        if (line[i] == '=') {
            eq = &line[i];
            break;
        }
    }

    if (eq) {
        *eq = '\0';                     // 切断 key/value
        vofa->on_cmd(line, parse_float(eq + 1));
    }
}

/* ======================================================================
 *  Zephyr async UART 回调（公开，供调用者注册）
 * ====================================================================== */

/**
 * @brief 扫描 DMA 缓冲区中的完整行，解析并回调 on_cmd
 *
 * 在 BUF_RELEASED / DISABLED 时自动 re-enable，调用者无需处理。
 */
void vofa_uart_cb(const struct device *dev, struct uart_event *evt,
                  void *user_data) {
    Vofa *vofa = user_data;

    switch (evt->type) {

    // ─── 有新数据到达 ───
    case UART_RX_RDY: {
        char  *buf = (char *)evt->data.rx.buf;
        size_t len = evt->data.rx.len;
        size_t line_start = 0;

        for (size_t i = 0; i < len; i++) {
            if (buf[i] == '\n') {
                process_line(vofa, &buf[line_start], i - line_start);
                line_start = i + 1;
            }
        }
        break;
    }

    // ─── DMA 缓冲区耗尽或超时停用，重新使能 ───
    case UART_RX_BUF_RELEASED:
    case UART_RX_DISABLED:
        uart_rx_enable(dev, vofa->rx_buf, vofa->rx_buf_size, SYS_FOREVER_US);
        break;

    default:
        break;
    }
}

/* ======================================================================
 *  公开 API
 * ====================================================================== */

/**
 * @brief 绑定 UART 设备，必须最先调用
 * @param vofa VOFA 实例指针
 * @param uart Zephyr UART 设备（如 DEVICE_DT_GET(DT_NODELABEL(usart10))）
 */
void vofa_init(Vofa *vofa, const struct device *uart) {
    vofa->uart = uart;
    uart_callback_set(uart, vofa_uart_cb, vofa);
}

/**
 * @brief 以 VOFA+ JustFloat 格式发送 float 数组（异步 DMA）
 *
 * 将 @p num 个 float 与帧尾 0x7F800000 (+inf) 拼进 static buffer，
 * 用一次 uart_tx 交给 DMA 异步发送，函数立即返回。
 *
 * @param vofa VOFA  实例指针
 * @param data float 数组首地址
 * @param num  uint8 个数
 */
void vofa_send(Vofa *vofa, const float *data, uint8_t num) {
    static __nocache uint8_t buf[VOFA_MAX_FLOATS * sizeof(float) + 4];
    size_t n = num * sizeof(float);
    const uint8_t *src = (const uint8_t *)data;

    for (size_t i = 0; i < n; i++) {   // 手动拷贝，避免依赖 memcpy
        buf[i] = src[i];
    }
    buf[n + 0] = 0x00;                 // 帧尾 0x7F800000（小端）
    buf[n + 1] = 0x00;
    buf[n + 2] = 0x80;
    buf[n + 3] = 0x7F;

    uart_tx(vofa->uart, buf, n + 4, SYS_FOREVER_US);
}

/**
 * @brief 设置 RX 缓冲区与命令回调
 *
 * 将 buf 和 on_cmd 存入 vofa 实例，供 vofa_uart_cb 内部使用。
 * 在此之后调用者自行注册 vofa_uart_cb 并启动 DMA。
 *
 * @param vofa         VOFA 实例指针
 * @param rx_buf       DMA 缓冲区
 * @param rx_buf_size  缓冲区大小
 * @param on_cmd       命令回调
 */
void vofa_set_handler(Vofa *vofa, uint8_t *rx_buf, size_t rx_buf_size,
                      vofa_cmd_handler on_cmd) {
    vofa->rx_buf      = rx_buf;
    vofa->rx_buf_size = rx_buf_size;
    vofa->on_cmd      = on_cmd;
}
