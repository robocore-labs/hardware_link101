/*
 * RoboCore Link101 — RP2350A pin map.
 *
 * Two kinds of assignment live here, and the difference matters:
 *
 *   FIXED    Something is soldered to this pin: a transceiver, a controller,
 *            a connector with one purpose. Every Link101 board is the same.
 *
 *   ASSIGNED Broken out to headers. What is on the other end is a property
 *            of the robot, not the board, so the application picks. The
 *            known conventions are recorded below but not #defined -- an
 *            application that wires four wheel motors to GP19..GP26 owns
 *            that decision and should say so in its own config.
 *
 * Board revisions differ by depopulation, not by remapping: rev2 drops the
 * RS485, CAN, and NeoPixel hardware. Nothing moves. So a rev2 firmware just
 * never includes link101/rs485.h, link101/can.h or link101/led.h, and the
 * fixed assignments below stay correct either way.
 */

#ifndef LINK101_PINS_H
#define LINK101_PINS_H

// ---- FIXED -----------------------------------------------------------

// RS485 transceiver (isolated, ADM2587E or similar). Aligns with hardware
// uart0 function select. Depopulated on rev2.
#define LINK101_PIN_RS485_TX      0
#define LINK101_PIN_RS485_RX      1
#define LINK101_PIN_RS485_DE      2   // direction enable

// Unified servo bus: Feetech STS/SCS and Dynamixel share one half-duplex
// pair with an external direction-enable line. Present on every revision.
#define LINK101_PIN_MOTOR_TX      7
#define LINK101_PIN_MOTOR_RX      8
#define LINK101_PIN_MOTOR_TXEN   16

// MCP2518FD CAN controller on spi1 (mikroBUS). Depopulated on rev2.
#define LINK101_PIN_CAN_INT       9   // polled, not wired to an IRQ
#define LINK101_PIN_SPI_SCK      10
#define LINK101_PIN_SPI_MOSI     11
#define LINK101_PIN_SPI_MISO     12
#define LINK101_PIN_SPI_CS       13

// I2C on i2c1: the Qwiic connector, and the two sensors fitted to the
// board. Their addresses are fixed by the layout (the LSM6DSOX has SDO/SA0
// tied high), so they are board facts like the pins themselves. Nothing
// else on the bus may use 0x6B or 0x30.
//
// Their INT lines go to solder jumpers JP1 (IMU) and JP2 (magnetometer),
// OPEN from the factory -- so nothing reaches a GPIO until you bridge one,
// and both drivers poll.
#define LINK101_PIN_SDA          14
#define LINK101_PIN_SCL          15
#define LINK101_LSM6DSOX_I2C_ADDR 0x6B   // see link101/lsm6dsox.h
#define LINK101_MMC5983_I2C_ADDR  0x30   // see link101/mmc5983.h

// Button. Read at boot to select config mode; otherwise general purpose.
#define LINK101_PIN_BUTTON       17

// NeoPixel activity LEDs, six on one data line. Depopulated on rev2.
#define LINK101_PIN_NEOPIXELS    18

// ---- ASSIGNED --------------------------------------------------------
//
// GP3..GP6 and GP19..GP26 are broken out. Conventions in use:
//
//   link101-fw-transparent   GP3/4 and GP5/6 as two general-purpose UART
//                            bridge channels; GP19/20 as an optional third.
//
//   link-base101             GP4/5 as RPLidar C1 on hardware uart1 (F2
//                            function select); GP19..GP26 as four DDSM210
//                            wheel motors, one PIO UART pair each. Loopback
//                            validated 6/6 on GP19->20, 21->22, 23->24,
//                            25->26.
//
// Both fit: every pin is GP0..GP29, so no PIO GPIO-base window shift is
// needed. State machines are claimed dynamically (see link101/pio_serial.h),
// so these uses no longer have to be reconciled by hand.

#endif // LINK101_PINS_H
