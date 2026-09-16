#include "link101/mmc5983.h"

#include "pico/stdlib.h"

// --- Register map ----------------------------------------------------------
#define REG_XOUT_0       0x00   // 0x00..0x06: X, Y, Z high bytes then the
                                // low 2 bits of each, packed into one byte
#define REG_STATUS       0x08
#define REG_CONTROL_0    0x09
#define REG_CONTROL_1    0x0A
#define REG_CONTROL_2    0x0B
#define REG_PRODUCT_ID   0x2F

#define PRODUCT_ID_VALUE 0x30

#define CONTROL_0_AUTO_SR_EN 0x20   // chip handles SET/RESET itself
#define CONTROL_1_SW_RST     0x80
#define CONTROL_2_CMM_EN     0x08   // continuous measurement
#define CONTROL_2_CM_100HZ   0x05   // CM_Freq = 100 Hz

// 18-bit output: unsigned, centred on 2^17, 16384 counts per gauss.
#define COUNTS_ZERO_FIELD  131072.0f
#define COUNTS_PER_GAUSS   16384.0f
#define GAUSS_TO_TESLA     1.0e-4f

// See the identical comment in lsm6dsox.c: the plain _blocking calls have
// no timeout, and a wedged bus hangs main() -- and USB with it.
#define I2C_TIMEOUT_US 1000

static bool reg_write(link101_mmc5983_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_timeout_us(dev->i2c, dev->addr, buf, 2, false, I2C_TIMEOUT_US) == 2;
}

static bool reg_read(link101_mmc5983_t *dev, uint8_t reg, uint8_t *dst, size_t n) {
    if (i2c_write_timeout_us(dev->i2c, dev->addr, &reg, 1, true, I2C_TIMEOUT_US) != 1) {
        return false;
    }
    return i2c_read_timeout_us(dev->i2c, dev->addr, dst, n, false, I2C_TIMEOUT_US) == (int)n;
}

bool link101_mmc5983_init(link101_mmc5983_t *dev, i2c_inst_t *i2c, uint8_t addr) {
    dev->i2c  = i2c;
    dev->addr = addr;

    uint8_t id = 0;
    if (!reg_read(dev, REG_PRODUCT_ID, &id, 1) || id != PRODUCT_ID_VALUE) {
        dev->i2c = NULL;      // nothing there; every read will fail cleanly
        return false;
    }

    if (!reg_write(dev, REG_CONTROL_1, CONTROL_1_SW_RST)) {
        return false;
    }
    sleep_ms(15);             // the reset takes ~10 ms

    // Automatic set/reset first, so every measurement the continuous mode
    // takes is already offset-corrected.
    if (!reg_write(dev, REG_CONTROL_0, CONTROL_0_AUTO_SR_EN) ||
        !reg_write(dev, REG_CONTROL_2, CONTROL_2_CMM_EN | CONTROL_2_CM_100HZ)) {
        return false;
    }

    sleep_ms(20);             // let the first measurements land
    return true;
}

bool link101_mmc5983_read(link101_mmc5983_t *dev, float tesla[3]) {
    if (dev->i2c == NULL) {
        return false;
    }

    uint8_t raw[7];
    if (!reg_read(dev, REG_XOUT_0, raw, sizeof(raw))) {
        return false;
    }

    // Each axis is 18 bits: two bytes of it in raw[0..5], and the last two
    // bits packed two-per-axis into raw[6], X in bits 7:6, Y in 5:4, Z in 3:2.
    for (int i = 0; i < 3; i++) {
        uint32_t counts = ((uint32_t)raw[2 * i] << 10) |
                          ((uint32_t)raw[2 * i + 1] << 2) |
                          ((uint32_t)(raw[6] >> (6 - 2 * i)) & 0x03);
        if (tesla) {
            tesla[i] = ((float)counts - COUNTS_ZERO_FIELD) / COUNTS_PER_GAUSS * GAUSS_TO_TESLA;
        }
    }
    return true;
}
