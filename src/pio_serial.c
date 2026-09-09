#include "link101/pio_serial.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

// Each bit is 8 PIO cycles (see the delay slots in uart_tx.pio/uart_rx.pio).
#define CYCLES_PER_BIT 8

static inline link101_pio_serial_t *self(serial_t *s) {
    return (link101_pio_serial_t *)s;  // serial_t is the first member
}

static uint32_t pio_serial_write(serial_t *s, const uint8_t *src, uint32_t len) {
    link101_pio_serial_t *p = self(s);
    if (!p->initialized || len == 0) return 0;

    if (p->txen_pin != LINK101_NO_TXEN) {
        gpio_put((uint)p->txen_pin, 1);  // drive the bus
    }

    for (uint32_t i = 0; i < len; i++) {
        pio_sm_put_blocking(p->pio_tx, p->sm_tx, (uint32_t)src[i]);
    }

    while (!pio_sm_is_tx_fifo_empty(p->pio_tx, p->sm_tx)) {
        tight_loop_contents();
    }

    if (p->txen_pin != LINK101_NO_TXEN) {
        // pio_sm_is_tx_fifo_empty reports the FIFO, not the output shift
        // register, so the last byte is still going out. Releasing TXEN now
        // truncates it on the wire. Hold for ~2 byte times, then let go.
        uint32_t byte_time_us = (10u * 1000000u) / p->baudrate;
        if (byte_time_us < 10) byte_time_us = 10;
        sleep_us(byte_time_us * 2);
        gpio_put((uint)p->txen_pin, 0);  // release the bus for RX
    }
    return len;
}

static uint32_t pio_serial_read(serial_t *s, uint8_t *dst, uint32_t max_len) {
    link101_pio_serial_t *p = self(s);
    if (!p->initialized) return 0;

    uint32_t count = 0;
    while (count < max_len && p->head != p->tail) {
        dst[count++] = p->buf[p->tail];
        p->tail = (p->tail + 1) % LINK101_PIO_SERIAL_RX_BUF;
    }
    return count;
}

static uint32_t pio_serial_available(serial_t *s) {
    link101_pio_serial_t *p = self(s);
    if (!p->initialized) return 0;
    return (p->head - p->tail + LINK101_PIO_SERIAL_RX_BUF) % LINK101_PIO_SERIAL_RX_BUF;
}

static void pio_serial_task(serial_t *s) {
    link101_pio_serial_t *p = self(s);
    if (!p->initialized) return;

    while (!pio_sm_is_rx_fifo_empty(p->pio_rx, p->sm_rx)) {
        uint8_t c = (uint8_t)(pio_sm_get(p->pio_rx, p->sm_rx) >> 24);
        uint32_t next_head = (p->head + 1) % LINK101_PIO_SERIAL_RX_BUF;
        if (next_head != p->tail) {
            p->buf[p->head] = c;
            p->head = next_head;
        }
        // else: buffer full, drop. Keep draining so the FIFO cannot stall.
    }
}

static bool pio_serial_set_baudrate(serial_t *s, uint32_t baudrate) {
    link101_pio_serial_t *p = self(s);
    if (!p->initialized || baudrate == 0) return false;
    if (baudrate == p->baudrate) return true;

    float div = (float)clock_get_hz(clk_sys) / (float)(baudrate * CYCLES_PER_BIT);

    pio_sm_set_enabled(p->pio_tx, p->sm_tx, false);
    pio_sm_set_enabled(p->pio_rx, p->sm_rx, false);
    pio_sm_clear_fifos(p->pio_tx, p->sm_tx);
    pio_sm_clear_fifos(p->pio_rx, p->sm_rx);
    pio_sm_set_clkdiv(p->pio_tx, p->sm_tx, div);
    pio_sm_set_clkdiv(p->pio_rx, p->sm_rx, div);
    pio_sm_set_enabled(p->pio_tx, p->sm_tx, true);
    pio_sm_set_enabled(p->pio_rx, p->sm_rx, true);

    p->baudrate = baudrate;
    p->head = p->tail = 0;
    return true;
}

static const serial_type_t pio_serial_type = {
    .write        = pio_serial_write,
    .read         = pio_serial_read,
    .available    = pio_serial_available,
    .task         = pio_serial_task,
    .set_baudrate = pio_serial_set_baudrate,
};

serial_t *link101_pio_serial_init(link101_pio_serial_t *p,
                                  uint tx_pin, uint rx_pin, int txen_pin,
                                  uint32_t baudrate) {
    if (p == NULL || baudrate == 0) return NULL;

    *p = (link101_pio_serial_t){0};
    p->base.type  = &pio_serial_type;
    p->txen_pin   = txen_pin;
    p->baudrate   = baudrate;

    // Let the SDK pick the block and state machine. Restricting the GPIO
    // range to the pin we actually drive lets it place the program in a
    // block whose GPIO base already covers it, on parts where that matters.
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(
            &pio_uart_tx_program, &p->pio_tx, &p->sm_tx, &p->off_tx,
            tx_pin, 1, true)) {
        return NULL;
    }
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(
            &pio_uart_rx_program, &p->pio_rx, &p->sm_rx, &p->off_rx,
            rx_pin, 1, true)) {
        pio_remove_program_and_unclaim_sm(&pio_uart_tx_program,
                                          p->pio_tx, p->sm_tx, p->off_tx);
        return NULL;
    }

    if (txen_pin != LINK101_NO_TXEN) {
        gpio_init((uint)txen_pin);
        gpio_set_dir((uint)txen_pin, GPIO_OUT);
        gpio_put((uint)txen_pin, 0);  // default to listening
    }

    pio_uart_tx_program_init(p->pio_tx, p->sm_tx, p->off_tx, tx_pin, baudrate);
    pio_uart_rx_program_init(p->pio_rx, p->sm_rx, p->off_rx, rx_pin, baudrate);

    p->initialized = true;
    return &p->base;
}

void link101_pio_serial_deinit(link101_pio_serial_t *p) {
    if (p == NULL || !p->initialized) return;
    pio_remove_program_and_unclaim_sm(&pio_uart_tx_program, p->pio_tx, p->sm_tx, p->off_tx);
    pio_remove_program_and_unclaim_sm(&pio_uart_rx_program, p->pio_rx, p->sm_rx, p->off_rx);
    p->initialized = false;
}
