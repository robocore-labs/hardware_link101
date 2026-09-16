/*
 * LSM6DSOX 6-axis IMU — the accelerometer and gyroscope fitted to the board.
 *
 * On I2C (i2c1, the same bus as the Qwiic connector) with SDO/SA0 tied high,
 * so it answers at 0x6B. Output is converted to ROS units on read: m/s^2
 * including gravity, and rad/s.
 *
 * This is a RAW 6-axis sensor. It does no sensor fusion and reports no
 * orientation -- unlike a BNO055, there is no quaternion to publish. Pair it
 * with the magnetometer (link101/mmc5983.h) and fuse on the host
 * (imu_filter_madgwick, robot_localization) if you need attitude.
 */

#ifndef LINK101_LSM6DSOX_H
#define LINK101_LSM6DSOX_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

// SDO/SA0 selects the low bit of the address. The board ties it high.
#define LINK101_LSM6DSOX_ADDR      0x6B
#define LINK101_LSM6DSOX_ADDR_ALT  0x6A

typedef struct {
    i2c_inst_t *i2c;
    uint8_t     addr;
    float       accel_scale;   // LSB -> m/s^2, set by init from the chosen range
    float       gyro_scale;    // LSB -> rad/s
} link101_lsm6dsox_t;

/*
 * Check the chip id and start both sensors at 104 Hz, +/-4 g and
 * +/-500 dps, with block data update on so a sample can't be read half
 * updated. The I2C bus must already be initialised.
 *
 * Returns false if the chip does not answer with the expected id.
 */
bool link101_lsm6dsox_init(link101_lsm6dsox_t *dev, i2c_inst_t *i2c, uint8_t addr);

/*
 * One sample: acceleration in m/s^2 (gravity included, as sensor_msgs/Imu
 * expects) and angular velocity in rad/s. Both are read in a single burst,
 * so the two vectors are from the same instant. Either pointer may be NULL.
 */
bool link101_lsm6dsox_read(link101_lsm6dsox_t *dev, float accel[3], float gyro[3]);

// Die temperature in degrees C. Measures the chip, not the room.
bool link101_lsm6dsox_read_temperature(link101_lsm6dsox_t *dev, float *celsius);

#endif // LINK101_LSM6DSOX_H
