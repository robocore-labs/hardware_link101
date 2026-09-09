/*
 * WS2812 (NeoPixel) driver for the board's activity LED strip.
 *
 * The board carries six pixels on one data line. What each pixel *means* --
 * which bus lights which LED -- is firmware policy, not a board fact, so it
 * stays in the application; this is just the strip.
 */

#ifndef LINK101_NEOPIXEL_H
#define LINK101_NEOPIXEL_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/pio.h"

#define LINK101_NEOPIXEL_COUNT 6          // pixels fitted on the board
#define LINK101_NEOPIXEL_FREQ  800000u    // WS2812B data rate

typedef struct { uint8_t r, g, b; } link101_rgb_t;

/*
 * Claim a state machine and drive num_pixels on the given pin. The pixel
 * buffer is caller-owned and must hold at least num_pixels entries -- the
 * original driver malloc'd it, which a library has no business doing.
 * Returns false if no state machine was free.
 */
bool link101_neopixel_init(uint pin, link101_rgb_t *pixels, uint num_pixels,
                           uint32_t frequency);

void link101_neopixel_set(uint index, link101_rgb_t color);
void link101_neopixel_fill(link101_rgb_t color);
void link101_neopixel_clear(void);

// Push the buffer out to the strip.
void link101_neopixel_show(void);

bool link101_neopixel_set_frequency(uint32_t frequency);

#endif // LINK101_NEOPIXEL_H
