/*
    MMC5603 Driver
*/

#include <math.h>
#include <zephyr/logging/log.h>
#include "MMC5603.h"

// 30G Range, 16-bit effective reading in this driver
// 1 bit = 1 mG (based on 0.0625mG for 20-bit, 16-bit >> 4)

extern int lsm_ext_write(const uint8_t addr, const uint8_t *buf, uint32_t num_bytes);
extern int lsm_ext_write_read(const uint8_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);

static int mmc_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return lsm_ext_write(0x30, buf, 2);
}

//static int mmc_read_reg(uint8_t reg, uint8_t *val) {
//    return lsm_ext_write_read(0x30, &reg, 1, val, 1);
//}

static int mmc_burst_read(uint8_t reg, uint8_t *buf, size_t len) {
    return lsm_ext_write_read(0x30, &reg, 1, buf, len);
}

static const float sensitivity = 0.001f; // 1 mG/LSB = 0.001 G/LSB
static const float offset = 32768.0f;    // Unsigned to Signed offset (2^16 / 2)

static int64_t oneshot_trigger_time = 0;

LOG_MODULE_REGISTER(MMC5603, LOG_LEVEL_DBG);

int mmc5603_init(float time, float *actual_time)
{
    int err = 0;

    // 1. Software Reset
    // Ignore the return error on reset, as the chip reboots instantly and may NACK
    mmc_write_reg(MMC5603_CTRL1_REG, 0x80);
    
    // Wait for OTP memory to reload and chip to boot
    k_sleep(K_MSEC(20)); 
    
    // Manual SET Pulse
    err = mmc_write_reg(MMC5603_CTRL0_REG, 0x08);
    if (err) LOG_ERR("Communication error in init (Manual SET)");
    
    // Wait 1ms for the hardware pulse to physically settle
    k_sleep(K_MSEC(1));

    // Enable Auto Set/Reset for continuous operation
    err = mmc_write_reg(MMC5603_CTRL0_REG, 0x20);
    if (err) LOG_ERR("Communication error in init (Auto SR)");

    *actual_time = INFINITY; // oneshot-only mode through I2C master
    return err;
}

void mmc5603_shutdown(void)
{
    mmc_write_reg(MMC5603_CTRL1_REG, 0x80);
}

int mmc5603_update_odr(float time, float *actual_time)
{
    // MMC5603 is behind the LSM6DSV I2C master which turns off between
    // transactions, so continuous measurement mode is not possible.
    // Always return INFINITY to signal oneshot-only mode.
    *actual_time = INFINITY;
    return 0;
}

void mmc5603_mag_oneshot(void)
{
    int err = mmc_write_reg(MMC5603_CTRL0_REG, 0x21);
    oneshot_trigger_time = k_uptime_get();
    if (err) LOG_ERR("Communication error in oneshot");
}

void mmc5603_mag_read(float m[3])
{
    int err = 0;
    uint8_t rawData[6];

    // Read the 6 data registers only (XOUT_0 through ZOUT_1)
    err = mmc_burst_read(MMC5603_XOUT_0, rawData, 6);
    
    if (err) {
        LOG_ERR("Communication error read (err=%d)", err);
    } else {
        mmc5603_mag_process(rawData, m);
    }
}

void mmc5603_mag_process(uint8_t *rawData, float m[3])
{
    // 1. Combine Big-Endian bytes into unsigned 16-bit integers
    uint16_t x_raw = (uint16_t)((rawData[0] << 8) | rawData[1]);
    uint16_t y_raw = (uint16_t)((rawData[2] << 8) | rawData[3]);
    uint16_t z_raw = (uint16_t)((rawData[4] << 8) | rawData[5]);

    // 2. Subtract the 32768 Null Field offset to center around 0
    // 3. Multiply by the 16-bit sensitivity 0.001 to get standard units
    m[0] = ((float)x_raw - offset) * sensitivity;
    m[1] = ((float)y_raw - offset) * sensitivity;
    m[2] = ((float)z_raw - offset) * sensitivity;

    //LOG_DBG("MMC5603 raw: %02X %02X %02X %02X %02X %02X -> %u %u %u -> %.2f %.2f %.2f",
    //        rawData[0], rawData[1], rawData[2], rawData[3], rawData[4], rawData[5],
    //        x_raw, y_raw, z_raw,
    //        (double)m[0], (double)m[1], (double)m[2]);
}

const sensor_mag_t sensor_mag_mmc5603 = {
    *mmc5603_init,
    *mmc5603_shutdown,
    *mmc5603_update_odr, 
    *mmc5603_mag_oneshot,
    *mmc5603_mag_read,
    NULL, // No temp read implemented yet
    *mmc5603_mag_process,
    6, 7 // 6 bytes read
};
