#include "link101/uart_serial.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

// An RS485 transceiver needs a moment to turn around before the first bit
// and after the last one, or the edges get clipped.
#define DE_SETTLE_US 5

static inline link101_uart_serial_t *self(serial_t *s) {
    return (link101_uart_serial_t *)s;  // serial_t is the first member
}

// The UART interrupt carries no context, so each instance registers itself
// here for its handler to find.
static link101_uart_serial_t *slot[2];

static void drain_into_ring(link101_uart_serial_t *p) {
    while (uart_is_readable(p->uart)) {
        uint8_t c = (uint8_t)uart_getc(p->uart);
        uint32_t next_head = (p->head + 1) % p->buf_len;
        if (next_head != p->tail) {
            p->buf[p->head] = c;
            p->head = next_head;
        }
        // else: ring full, drop. Keep reading so the FIFO cannot wedge.
    }
}

static void uart0_rx_irq(void) { if (slot[0]) drain_into_ring(slot[0]); }
static void uart1_rx_irq(void) { if (slot[1]) drain_into_ring(slot[1]); }

static uint32_t uart_serial_write(serial_t *s, const uint8_t *src, uint32_t len) {
    link101_uart_serial_t *p = self(s);
    if (!p->initialized || len == 0) return 0;

    if (p->de_pin != LINK101_NO_DE) {
        gpio_put((uint)p->de_pin, 1);
        busy_wait_us(DE_SETTLE_US);
    }

    uart_write_blocking(p->uart, src, len);

    if (p->de_pin != LINK101_NO_DE) {
        uart_tx_wait_blocking(p->uart);  // shift register, not just the FIFO
        busy_wait_us(DE_SETTLE_US);
        gpio_put((uint)p->de_pin, 0);
    }
    return len;
}

static uint32_t uart_serial_read(serial_t *s, uint8_t *dst, uint32_t max_len) {
    link101_uart_serial_t *p = self(s);
    if (!p->initialized) return 0;

    uint32_t count = 0;
    while (count < max_len && p->head != p->tail) {
        dst[count++] = p->buf[p->tail];
        p->tail = (p->tail + 1) % p->buf_len;
    }
    return count;
}

static uint32_t uart_serial_available(serial_t *s) {
    link101_uart_serial_t *p = self(s);
    if (!p->initialized) return 0;
    return (p->head - p->tail + p->buf_len) % p->buf_len;
}

// RX arrives on the interrupt, so there is nothing to pump. Kept non-NULL so
// an application can still wrap it to chain tud_task() across a blocking wait.
static void uart_serial_task(serial_t *s) {
    (void)s;
}

static bool uart_serial_set_baudrate(serial_t *s, uint32_t baudrate) {
    link101_uart_serial_t *p = self(s);
    if (!p->initialized || baudrate == 0) return false;
    if (baudrate == p->baudrate) return true;
    uart_set_baudrate(p->uart, baudrate);
    p->baudrate = baudrate;
    return true;
}

static const serial_type_t uart_serial_type = {
    .write        = uart_serial_write,
    .read         = uart_serial_read,
    .available    = uart_serial_available,
    .task         = uart_serial_task,
    .set_baudrate = uart_serial_set_baudrate,
};

serial_t *link101_uart_serial_init(link101_uart_serial_t *p,
                                   uart_inst_t *uart,
                                   uint tx_pin, uint rx_pin, int de_pin,
                                   uint32_t baudrate,
                                   uint8_t *buf, uint32_t buf_len) {
    if (p == NULL || uart == NULL || buf == NULL || buf_len < 2 || baudrate == 0) {
        return NULL;
    }

    *p = (link101_uart_serial_t){0};
    p->base.type = &uart_serial_type;
    p->uart      = uart;
    p->de_pin    = de_pin;
    p->baudrate  = baudrate;
    p->buf       = buf;
    p->buf_len   = buf_len;

    if (de_pin != LINK101_NO_DE) {
        gpio_init((uint)de_pin);
        gpio_set_dir((uint)de_pin, GPIO_OUT);
        gpio_put((uint)de_pin, 0);  // default to listening
    }

    uart_init(uart, baudrate);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart, false, false);
    uart_set_fifo_enabled(uart, true);

    uint idx = uart_get_index(uart);
    slot[idx] = p;
    irq_set_exclusive_handler(idx == 0 ? UART0_IRQ : UART1_IRQ,
                              idx == 0 ? uart0_rx_irq : uart1_rx_irq);
    irq_set_enabled(idx == 0 ? UART0_IRQ : UART1_IRQ, true);
    uart_set_irq_enables(uart, true, false);  // RX only

    p->initialized = true;
    return &p->base;
}

void link101_uart_serial_deinit(link101_uart_serial_t *p) {
    if (p == NULL || !p->initialized) return;
    uint idx = uart_get_index(p->uart);
    uart_set_irq_enables(p->uart, false, false);
    irq_set_enabled(idx == 0 ? UART0_IRQ : UART1_IRQ, false);
    slot[idx] = NULL;
    uart_deinit(p->uart);
    p->initialized = false;
}
