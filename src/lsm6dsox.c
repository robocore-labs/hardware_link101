#include "link101/lsm6dsox.h"

#include "pico/stdlib.h"

// --- Register map ----------------------------------------------------------
#define REG_WHO_AM_I     0x0F
#define REG_CTRL1_XL     0x10   // accelerometer: ODR, full scale
#define REG_CTRL2_G      0x11   // gyroscope: ODR, full scale
#define REG_CTRL3_C      0x12   // BDU, IF_INC, software reset
#define REG_OUT_TEMP_L   0x20   // temperature, then gyro, then accel:
#define REG_OUTX_L_G     0x22   // 0x20..0x2D is one contiguous block
#define REG_OUTX_L_A     0x28


#define CTRL3_C_BOOT     0x80
#define CTRL3_C_BDU      0x40   // don't update a sample mid-read
#define CTRL3_C_IF_INC   0x04   // auto-increment on multi-byte reads
#define CTRL3_C_SW_RESET 0x01

// ODR nibble (bits 7:4) shared by CTRL1_XL and CTRL2_G. 104 Hz is twice the
// 50 Hz a robot control loop typically publishes at -- enough that a sample
// is never stale, low enough to stay quiet.
#define ODR_104_HZ       0x40

// Accelerometer full scale (CTRL1_XL bits 3:2). The encoding is not in
// numerical order: 00 = 2 g, 01 = 16 g, 10 = 4 g, 11 = 8 g.
#define FS_XL_4G         0x08
#define LSB_PER_G_4G     8192.0f          // 0.122 mg/LSB
#define G_TO_MS2         9.80665f

// Gyroscope full scale (CTRL2_G bits 3:2): 00 = 250, 01 = 500, 10 = 1000,
// 11 = 2000 dps. 500 dps is ~1.4 turns/second, well past what a wheeled base
// does, and keeps four times the resolution of the 2000 dps range.
#define FS_G_500DPS      0x04
#define LSB_PER_DPS_500  57.14286f        // 17.5 mdps/LSB
#define DEG_TO_RAD       0.01745329251994329577f

// Temperature: 256 LSB per degree, zero at 25 C.
#define LSB_PER_DEGREE   256.0f
#define TEMP_ZERO_C      25.0f

// Timeouts, not the plain _blocking calls: those have none, and a bus
// wedged by a chip that never releases SDA (or one that just isn't there,
// on a board revision without it) hangs main() forever -- taking USB down
// with it, since nothing pumps tud_task() while this call sits. A few
// hundred us is generous for a handful of bytes at 400 kHz.
#define I2C_TIMEOUT_US 1000

static bool reg_write(link101_lsm6dsox_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_timeout_us(dev->i2c, dev->addr, buf, 2, false, I2C_TIMEOUT_US) == 2;
}

static bool reg_read(link101_lsm6dsox_t *dev, uint8_t reg, uint8_t *dst, size_t n) {
    if (i2c_write_timeout_us(dev->i2c, dev->addr, &reg, 1, true, I2C_TIMEOUT_US) != 1) {
        return false;
    }
    return i2c_read_timeout_us(dev->i2c, dev->addr, dst, n, false, I2C_TIMEOUT_US) == (int)n;
}

static inline int16_t le16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

bool link101_lsm6dsox_probe(i2c_inst_t *i2c, uint8_t addr, uint8_t *chip_id) {
    // Raw, not through reg_read(): that takes a link101_lsm6dsox_t*, and
    // this runs precisely when there may be no valid device to build one
    // from -- the whole point is finding out.
    uint8_t reg = REG_WHO_AM_I;
    uint8_t id  = 0;
    bool acked = i2c_write_timeout_us(i2c, addr, &reg, 1, true, I2C_TIMEOUT_US) == 1 &&
                 i2c_read_timeout_us(i2c, addr, &id, 1, false, I2C_TIMEOUT_US) == 1;
    if (chip_id) {
        *chip_id = id;
    }
    return acked;
}

bool link101_lsm6dsox_init(link101_lsm6dsox_t *dev, i2c_inst_t *i2c, uint8_t addr) {
    dev->i2c  = i2c;
    dev->addr = addr;

    uint8_t id = 0;
    if (!reg_read(dev, REG_WHO_AM_I, &id, 1) || id != LINK101_LSM6DSOX_WHO_AM_I) {
        dev->i2c = NULL;      // nothing there; every read will fail cleanly
        return false;
    }

    // Reset, then wait for the chip to come back. The bit self-clears.
    if (!reg_write(dev, REG_CTRL3_C, CTRL3_C_SW_RESET)) {
        return false;
    }
    for (int i = 0; i < 20; i++) {
        sleep_ms(1);
        uint8_t ctrl3 = 0;
        if (reg_read(dev, REG_CTRL3_C, &ctrl3, 1) && !(ctrl3 & CTRL3_C_SW_RESET)) {
            break;
        }
    }

    if (!reg_write(dev, REG_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC) ||
        !reg_write(dev, REG_CTRL1_XL, ODR_104_HZ | FS_XL_4G) ||
        !reg_write(dev, REG_CTRL2_G,  ODR_104_HZ | FS_G_500DPS)) {
        return false;
    }

    dev->accel_scale = G_TO_MS2 / LSB_PER_G_4G;
    dev->gyro_scale  = DEG_TO_RAD / LSB_PER_DPS_500;

    sleep_ms(20);   // first samples are ready after a couple of ODR periods
    return true;
}

bool link101_lsm6dsox_read(link101_lsm6dsox_t *dev, float accel[3], float gyro[3]) {
    if (dev->i2c == NULL) {
        return false;
    }

    // Gyro (0x22) and accel (0x28) are contiguous, so one burst gets both
    // from the same instant rather than two transactions apart.
    uint8_t raw[12];
    if (!reg_read(dev, REG_OUTX_L_G, raw, sizeof(raw))) {
        return false;
    }

    for (int i = 0; i < 3; i++) {
        if (gyro) {
            gyro[i] = le16(&raw[2 * i]) * dev->gyro_scale;
        }
        if (accel) {
            accel[i] = le16(&raw[6 + 2 * i]) * dev->accel_scale;
        }
    }
    return true;
}

bool link101_lsm6dsox_read_temperature(link101_lsm6dsox_t *dev, float *celsius) {
    if (dev->i2c == NULL) {
        return false;
    }

    uint8_t raw[2];
    if (!reg_read(dev, REG_OUT_TEMP_L, raw, sizeof(raw))) {
        return false;
    }
    if (celsius) {
        *celsius = TEMP_ZERO_C + le16(raw) / LSB_PER_DEGREE;
    }
    return true;
}
