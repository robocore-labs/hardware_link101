/*
 * A PIO-backed UART presented as a serial_t.
 *
 * This replaces two near-identical files in the firmware repos:
 * half_duplex.c (one port, with a TX-enable line) and ddsm_port.c (four
 * ports, without). They had the same ring buffer, the same FIFO drain and
 * the same blocking write; the only real difference was direction control
 * and how many instances existed. Both are now this type -- half duplex is
 * just a port that owns a TXEN pin.
 *
 * State machines are claimed from the SDK rather than hardcoded. The old
 * code assigned PIO blocks by #define, which is why the NeoPixel driver
 * (pio2 SM0) and the fourth wheel motor (pio2 SM0/SM1) could not coexist in
 * one binary. Nothing here knows or cares which block it gets.
 */

#ifndef LINK101_PIO_SERIAL_H
#define LINK101_PIO_SERIAL_H

#include "pico_serial.h"
#include "hardware/pio.h"

#ifndef LINK101_PIO_SERIAL_RX_BUF
#define LINK101_PIO_SERIAL_RX_BUF 256   // must be a power of two
#endif

#define LINK101_NO_TXEN (-1)

typedef struct {
    serial_t base;              // must be first: implementations cast to it

    PIO      pio_tx, pio_rx;
    uint     sm_tx,  sm_rx;
    uint     off_tx, off_rx;

    int      txen_pin;          // LINK101_NO_TXEN for a full-duplex port
    uint32_t baudrate;
    bool     initialized;

    uint8_t  buf[LINK101_PIO_SERIAL_RX_BUF];
    volatile uint32_t head, tail;
} link101_pio_serial_t;

/*
 * Bring up a PIO UART on the given pins and return it as a serial port.
 *
 * txen_pin drives an external transceiver's direction line and is held
 * asserted until the last stop bit has left the shift register; pass
 * LINK101_NO_TXEN for a full-duplex port. Returns NULL if no state machine
 * or instruction memory was available.
 */
serial_t *link101_pio_serial_init(link101_pio_serial_t *p,
                                  uint tx_pin, uint rx_pin, int txen_pin,
                                  uint32_t baudrate);

// Release the claimed state machines and instruction memory.
void link101_pio_serial_deinit(link101_pio_serial_t *p);

#endif // LINK101_PIO_SERIAL_H
