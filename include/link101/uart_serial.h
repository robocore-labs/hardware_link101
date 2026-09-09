/*
 * A hardware-UART port presented as a serial_t.
 *
 * The board's other two byte pipes were the same file twice: rs485.c (uart0
 * plus a direction-enable pin, polled) and lidar_uart.c (uart1, IRQ-driven,
 * bigger buffer). Same ring buffer, same read/available, same 8N1 setup.
 * They are one type now, with the direction pin optional and RX always on
 * the interrupt -- polling was only ever fast enough because RS485 ran slow,
 * and a lidar at 460800 baud will overrun the 32-byte hardware FIFO if the
 * main loop stalls.
 *
 * uart_bridge.c is gone too, but it did not need replacing: it was N PIO
 * UARTs with ring buffers, which is link101_pio_serial_t. Make one per
 * channel.
 */

#ifndef LINK101_UART_SERIAL_H
#define LINK101_UART_SERIAL_H

#include "pico_serial.h"
#include "hardware/uart.h"

#define LINK101_NO_DE (-1)

typedef struct {
    serial_t base;              // must be first

    uart_inst_t *uart;
    int          de_pin;        // LINK101_NO_DE if the line is not shared
    uint32_t     baudrate;
    bool         initialized;

    uint8_t  *buf;              // caller-owned
    uint32_t  buf_len;
    volatile uint32_t head, tail;
} link101_uart_serial_t;

/*
 * Bring up a hardware UART on the given pins and return it as a serial port.
 *
 * The RX ring is supplied by the caller because the right size is a property
 * of the traffic: a couple of hundred bytes for request/response on RS485, a
 * couple of thousand for a lidar streaming scans. buf must outlive the port.
 *
 * de_pin drives a transceiver's direction line, asserted around writes with
 * the settling delays an RS485 transceiver needs; pass LINK101_NO_DE for a
 * plain UART. Returns NULL on bad arguments.
 */
serial_t *link101_uart_serial_init(link101_uart_serial_t *p,
                                   uart_inst_t *uart,
                                   uint tx_pin, uint rx_pin, int de_pin,
                                   uint32_t baudrate,
                                   uint8_t *buf, uint32_t buf_len);

void link101_uart_serial_deinit(link101_uart_serial_t *p);

#endif // LINK101_UART_SERIAL_H
