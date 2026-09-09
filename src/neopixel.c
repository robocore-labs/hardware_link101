#include "link101/neopixel.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"

#include "ws2812.pio.h"

// One strip on the board, so one instance. Unlike the serial ports there is
// nothing to parameterise across several of them.
static PIO      neo_pio;
static uint     neo_sm, neo_offset;
static uint     neo_pin;
static uint32_t neo_frequency;
static link101_rgb_t *neo_pixels;
static uint     neo_count;
static bool     neo_ready;

static inline uint32_t to_grb(link101_rgb_t c) {
    return ((uint32_t)c.g << 16) | ((uint32_t)c.r << 8) | (uint32_t)c.b;
}

bool link101_neopixel_init(uint pin, link101_rgb_t *pixels, uint num_pixels,
                           uint32_t frequency) {
    if (pixels == NULL || num_pixels == 0 || frequency == 0) return false;

    neo_pixels = pixels;
    neo_count  = num_pixels;
    neo_pin    = pin;

    // Improves signal integrity driving 5 V parts from 3.3 V logic.
    gpio_set_pulls(pin, true, false);

    // Claimed rather than hardcoded to pio2 SM0, which is what used to make
    // the LED strip and the fourth wheel motor mutually exclusive.
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(
            &ws2812_program, &neo_pio, &neo_sm, &neo_offset, pin, 1, true)) {
        return false;
    }

    neo_frequency = frequency;
    ws2812_program_init(neo_pio, neo_sm, neo_offset, pin, frequency, false);

    neo_ready = true;
    link101_neopixel_clear();
    link101_neopixel_show();
    return true;
}

void link101_neopixel_set(uint index, link101_rgb_t color) {
    if (!neo_ready || index >= neo_count) return;
    neo_pixels[index] = color;
}

void link101_neopixel_fill(link101_rgb_t color) {
    if (!neo_ready) return;
    for (uint i = 0; i < neo_count; i++) {
        neo_pixels[i] = color;
    }
}

void link101_neopixel_clear(void) {
    link101_neopixel_fill((link101_rgb_t){0, 0, 0});
}

void link101_neopixel_show(void) {
    if (!neo_ready) return;

    for (uint i = 0; i < neo_count; i++) {
        pio_sm_put_blocking(neo_pio, neo_sm, to_grb(neo_pixels[i]) << 8u);
        // Stagger the updates so all six pixels do not switch at once.
        if (i + 1 < neo_count) sleep_us(10);
    }
    sleep_us(500);  // latch
}

bool link101_neopixel_set_frequency(uint32_t frequency) {
    if (!neo_ready || frequency == 0) return false;
    neo_frequency = frequency;
    ws2812_program_init(neo_pio, neo_sm, neo_offset, neo_pin, frequency, false);
    return true;
}
