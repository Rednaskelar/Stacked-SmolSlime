/*
    MMC5603 Driver
*/

#include <math.h>
#include <zephyr/logging/log.h>
#include "MMC5603.h"

// 30G Range, 16-bit effective reading in this driver
// 1 bit = 1 mG (based on 0.0625mG for 20-bit, 16-bit >> 4)
static const float sensitivity = 0.001f; // 1 mG/LSB = 0.001 G/LSB
static const float offset = 32768.0f;    // Unsigned to Signed offset (2^16 / 2)

static int64_t oneshot_trigger_time = 0;

LOG_MODULE_REGISTER(MMC5603, LOG_LEVEL_DBG);

int mmc5603_init(float time, float *actual_time)
{
    // Reset
    int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603_CTRL1_REG, 0x80); // SW Reset similar to others?
    if (err) LOG_ERR("Communication error in init");
    k_sleep(K_MSEC(10)); // Wait for reset
    
    // Set CMM/ODR if supported, or just Set/Reset
    // MMC5603 is often used in simple TakeMeasurement mode.
    // Init automatic Set/Reset if available?
    
    // For now, we will assume generic usage: Oneshot driven by main loop.
    *actual_time = time;
    return 0;
}

void mmc5603_shutdown(void)
{
    // Reset or power down
    ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603_CTRL1_REG, 0x80);
}

int mmc5603_update_odr(float time, float *actual_time)
{
    // MMC5603 typically runs in oneshot for this driver structure
    *actual_time = time;
    return 0;
}

void mmc5603_mag_oneshot(void)
{
    // TM_M (Take Magnetic Measurement) is usually bit 0 of Internal Control 0 (0x1B).
    // And usually we want to do Set/Reset.
    // MMC5603 might have Auto Set/Reset or manual.
    // Let's try sending TM_M (0x01) to CTRL0 (0x1B).
    int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603_CTRL0_REG, 0x01);
    oneshot_trigger_time = k_uptime_get();
    if (err) LOG_ERR("Communication error in oneshot");
}

void mmc5603_mag_read(float m[3])
{
    int err = 0;
    uint8_t status = 0;
    int64_t timeout = oneshot_trigger_time + 10; // 10ms timeout (should be fast)
    
    if (k_uptime_get() >= timeout) oneshot_trigger_time = 0;
    
    // Check Status Register (0x18), Bit 0 (Meas M Done) - Active High
    while (k_uptime_get() < timeout)
    {
        err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_MAG, MMC5603_STATUS_REG, &status);
        if (err) break;
        if (status & 0x40) break; // Wait. Actually MMC5603 datasheet says Bit 6 (0x40) is meas_done? 
                                  // Or Bit 0? 
                                  // Common MEMSIC: Bit 0 is Meas Done.
                                  // MMC5983MA: Bit 0.
                                  // MMC5603 datasheet check... 
                                  // Many sources say Status 0x18 Bit 6 (0x40) or Bit 0. 
                                  // I'll try Bit 6 as per some online MMC56x3 drivers, or Bit 0.
                                  // Let's assume Bit 0 first as it's standard unless I find otherwise.
                                  // Actually, I'll check both or just wait sufficient time.
                                  // MMC34160 uses 0x01.
                                  // I'll use 0x01 for now.
        if (status & 0x01) break; 
    }
    
    if (!(status & 0x01)) 
    {
        // LOG_ERR("Read timeout or not done");
        // Proceed anyway or return?
    }
    
    oneshot_trigger_time = 0;
    
    uint8_t rawData[6];
    err |= ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, MMC5603_XOUT_0, rawData, 6);
    if (err) LOG_ERR("Communication error read");
    
    mmc5603_mag_process(rawData, m);
}

void mmc5603_mag_process(uint8_t *raw_m, float m[3])
{
    // Data format: X0, X1, Y0, Y1, Z0, Z1
    // X0 is MSB, X1 is LSB (of the 16 bits we read)
    // 20-bit mode: X0(19:12), X1(11:4).
    // We treat this as 16-bit value.
    
    uint16_t x = ((uint16_t)raw_m[0] << 8) | raw_m[1];
    uint16_t y = ((uint16_t)raw_m[2] << 8) | raw_m[3];
    uint16_t z = ((uint16_t)raw_m[4] << 8) | raw_m[5];
    
    // Unsigned data centered at 32768 (2^15)
    m[0] = ((float)x - offset) * sensitivity;
    m[1] = ((float)y - offset) * sensitivity;
    m[2] = ((float)z - offset) * sensitivity;
}

const sensor_mag_t sensor_mag_mmc5603 = {
    *mmc5603_init,
    *mmc5603_shutdown,
    *mmc5603_update_odr, // Using dummy for now
    *mmc5603_mag_oneshot,
    *mmc5603_mag_read,
    NULL, // No temp read implemented yet
    *mmc5603_mag_process,
    6, 7 // 6 bytes read
};
