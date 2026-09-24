#include "lib/vofa/vofa.h"

#include <errno.h>
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

BUILD_ASSERT(sizeof(float) == sizeof(uint32_t), "JustFloat requires 32-bit floats");

/* Parse the existing decimal key=value protocol after terminating the line. */
static float parse_float(const char *s) {
    float result = 0.0f;
    float sign = 1.0f;

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

    if (eq && vofa->on_cmd != NULL) {
        *eq = '\0'; // 切断 key/value
        vofa->on_cmd(line, parse_float(eq + 1));
    }
}

static void receive_byte(Vofa *vofa, uint8_t byte) {
    if (byte == '\n') {
        if (!vofa->rx_discard && vofa->rx_length != 0) {
            if (vofa->rx_buf[vofa->rx_length - 1] == '\r') {
                vofa->rx_length--;
            }
            vofa->rx_buf[vofa->rx_length] = '\0';
            process_line(vofa, (char *)vofa->rx_buf, vofa->rx_length);
        }
        vofa->rx_length = 0;
        vofa->rx_discard = false;
    }
    else if (!vofa->rx_discard) {
        if (byte == '\0' || vofa->rx_length >= vofa->rx_buf_size - 1) {
            /* Never interpret the tail of a truncated command as a new line. */
            vofa->rx_length = 0;
            vofa->rx_discard = true;
        }
        else {
            vofa->rx_buf[vofa->rx_length++] = byte;
        }
    }
}

static int transmit_ready(Vofa *vofa) {
    k_spinlock_key_t key = k_spin_lock(&vofa->tx_lock);
    int written = 0;

    if (vofa->tx_count != 0) {
        const uint8_t head = vofa->tx_head;
        written = uart_fifo_fill(vofa->uart, &vofa->tx_frames[head][vofa->tx_offset],
                                 vofa->tx_lengths[head] - vofa->tx_offset);
        if (written > 0) {
            vofa->tx_offset += written;
            if (vofa->tx_offset == vofa->tx_lengths[head]) {
                vofa->tx_head = (head + 1) % VOFA_TX_QUEUE_DEPTH;
                vofa->tx_count--;
                vofa->tx_offset = 0;
            }
        }
        else if (written < 0) {
            vofa->tx_error = written;
            vofa->tx_count = 0;
            vofa->tx_offset = 0;
        }
    }

    /* Serialize disabling TX with producers enabling it after enqueueing. */
    if (vofa->tx_count == 0) {
        uart_irq_tx_disable(vofa->uart);
    }
    k_spin_unlock(&vofa->tx_lock, key);
    return written;
}

static void vofa_irq_callback(const struct device *dev, void *user_data) {
    Vofa *vofa = user_data;

    while (true) {
        uart_irq_update(dev);
        if (!uart_irq_is_pending(dev)) {
            break;
        }
        bool progressed = false;

        if (uart_irq_rx_ready(dev)) {
            uint8_t bytes[32];
            int received;

            /* Drain RX until a short read, as required by the UART IRQ API. */
            do {
                received = uart_fifo_read(dev, bytes, sizeof(bytes));
                if (received > 0) {
                    progressed = true;
                    if (vofa->on_cmd != NULL) {
                        for (int i = 0; i < received; ++i) {
                            receive_byte(vofa, bytes[i]);
                        }
                    }
                }
            } while (received == sizeof(bytes));
        }

        if (uart_irq_tx_ready(dev)) {
            progressed |= transmit_ready(vofa) > 0;
        }
        if (!progressed) {
            break;
        }
    }
}

int vofa_init(Vofa *vofa, const struct device *uart) {
    if (vofa == NULL) {
        return -EINVAL;
    }

    /* Existing callers may supply uninitialized stack storage. */
    memset(vofa, 0, sizeof(*vofa));
    if (uart == NULL) {
        return -EINVAL;
    }
    if (!device_is_ready(uart)) {
        return -ENODEV;
    }

    vofa->uart = uart;
    int err = uart_irq_callback_user_data_set(uart, vofa_irq_callback, vofa);
    if (err != 0) {
        vofa->uart = NULL;
        return err;
    }
    uart_irq_tx_disable(uart);
    uart_irq_rx_disable(uart);
    return 0;
}

int vofa_send(Vofa *vofa, const float *data, uint8_t num) {
    if (vofa == NULL || data == NULL || num == 0 || num > VOFA_MAX_FLOATS) {
        return -EINVAL;
    }
    if (vofa->uart == NULL) {
        return -ENODEV;
    }

    uint8_t frame[VOFA_FRAME_SIZE];
    const uint8_t length = (num + 1) * sizeof(float);
    for (uint8_t i = 0; i < num; ++i) {
        uint32_t bits;
        memcpy(&bits, &data[i], sizeof(bits));
        sys_put_le32(bits, &frame[i * sizeof(float)]);
    }
    sys_put_le32(0x7f800000U, &frame[num * sizeof(float)]);

    k_spinlock_key_t key = k_spin_lock(&vofa->tx_lock);
    int err = vofa->tx_error;
    if (err == 0 && vofa->tx_count == VOFA_TX_QUEUE_DEPTH) {
        err = -ENOBUFS;
    }
    if (err == 0) {
        const uint8_t tail = (vofa->tx_head + vofa->tx_count) % VOFA_TX_QUEUE_DEPTH;
        memcpy(vofa->tx_frames[tail], frame, length);
        vofa->tx_lengths[tail] = length;
        vofa->tx_count++;
        uart_irq_tx_enable(vofa->uart);
    }
    k_spin_unlock(&vofa->tx_lock, key);
    return err;
}

int vofa_set_handler(Vofa *vofa, uint8_t *rx_buf, size_t rx_buf_size, vofa_cmd_handler on_cmd) {
    if (vofa == NULL || rx_buf == NULL || rx_buf_size < 2 || on_cmd == NULL) {
        return -EINVAL;
    }
    if (vofa->uart == NULL) {
        return -ENODEV;
    }
    if (vofa->rx_buf != NULL) {
        return -EALREADY;
    }

    vofa->rx_buf = rx_buf;
    vofa->rx_buf_size = rx_buf_size;
    vofa->rx_length = 0;
    vofa->rx_discard = false;
    vofa->on_cmd = on_cmd;
    uart_irq_rx_enable(vofa->uart);
    return 0;
}
