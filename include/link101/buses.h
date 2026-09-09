/*
 * Board-fixed bus constructors.
 *
 * Only buses whose pins are a property of the PCB belong here. Anything
 * wired through the breakout headers -- wheel motors, lidar, extra UART
 * channels -- is robot wiring, so the application calls
 * link101_pio_serial_init() with its own pins rather than finding a
 * constructor here that has guessed on its behalf.
 */

#ifndef LINK101_BUSES_H
#define LINK101_BUSES_H

#include "link101/pins.h"
#include "link101/pio_serial.h"
#include "link101/uart_serial.h"

/*
 * The unified servo bus: Feetech STS/SCS and Dynamixel on one half-duplex
 * pair with an external direction line. Present on every board revision.
 * Feetech STS runs at 1 Mbaud by default.
 */
static inline serial_t *link101_servo_bus_init(link101_pio_serial_t *p,
                                               uint32_t baudrate) {
    return link101_pio_serial_init(p,
                                   LINK101_PIN_MOTOR_TX,
                                   LINK101_PIN_MOTOR_RX,
                                   LINK101_PIN_MOTOR_TXEN,
                                   baudrate);
}

/*
 * The isolated RS485 transceiver on uart0, with its direction line.
 * Depopulated on rev2 boards. The caller owns the RX ring; a few hundred
 * bytes suits request/response traffic.
 */
static inline serial_t *link101_rs485_init(link101_uart_serial_t *p,
                                           uint32_t baudrate,
                                           uint8_t *buf, uint32_t buf_len) {
    return link101_uart_serial_init(p, uart0,
                                    LINK101_PIN_RS485_TX,
                                    LINK101_PIN_RS485_RX,
                                    LINK101_PIN_RS485_DE,
                                    baudrate, buf, buf_len);
}

#endif // LINK101_BUSES_H
