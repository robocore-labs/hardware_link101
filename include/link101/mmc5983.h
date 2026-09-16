/*
 * MMC5983MA magnetometer — the compass fitted to the board.
 *
 * On I2C (i2c1, the same bus as the IMU and the Qwiic connector) at its
 * fixed address 0x30. Output is converted to ROS units on read: tesla.
 *
 * Runs in continuous measurement mode, so a read is just a register read --
 * no trigger, no waiting for a conversion, nothing to block a control loop.
 *
 * A note on SET/RESET: this sensor's zero point drifts, and the way it is
 * corrected is to flip the internal magnetisation and measure again. The
 * chip can do that itself, and init() turns that on, so callers never have
 * to think about it.
 */

#ifndef LINK101_MMC5983_H
#define LINK101_MMC5983_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

// Fixed: the MMC5983MA has no address select pin.
#define LINK101_MMC5983_ADDR  0x30

typedef struct {
    i2c_inst_t *i2c;
    uint8_t     addr;
} link101_mmc5983_t;

/*
 * Check the product id, reset, enable automatic set/reset, and start
 * continuous measurement at 100 Hz. The I2C bus must already be
 * initialised.
 *
 * Returns false if the chip does not answer with the expected id.
 */
bool link101_mmc5983_init(link101_mmc5983_t *dev, i2c_inst_t *i2c, uint8_t addr);

/*
 * The most recent measurement, in tesla. Earth's field is 25-65 uT
 * (2.5e-5 to 6.5e-5 T) depending where you are, so a healthy reading has a
 * magnitude in that range once you are away from motors and speakers.
 */
bool link101_mmc5983_read(link101_mmc5983_t *dev, float tesla[3]);

#endif // LINK101_MMC5983_H
