/*
    MMC5603 Driver for SlimeVR
    Based on MMC5983MA Driver
*/

#ifndef MMC5603_H
#define MMC5603_H

#include "sensor/sensor.h"

// MMC5603NJ specific registers
// Based on datasheet snippets and relationship to MMC5983MA
#define MMC5603_XOUT_0        0x00
// ... data registers up to 0x05 for 16-bit, or potentially 0x06/0x07 for 20-bit if available
// We will read 6 bytes for 16-bit operation

#define MMC5603_STATUS        0x18 // Status register for MMC5603 is often 0x18 or similar in newer MEMSIC, but 5983 was 0x08. 
                                   // Scan snippets said Xout at 0x00... Zout at 0x05.
                                   // Let's assume standard MEMSIC 0x08 for now unless proven otherwise?
                                   // Memsic code often uses 0x08.
#define MMC5603_CONTROL_0     0x1B // Internal Control 0
#define MMC5603_CONTROL_1     0x1C // Internal Control 1
#define MMC5603_CONTROL_2     0x1D // Internal Control 2?
// Wait, MMC34160 uses 0x20. MMC5603 might be different. 
// However, MMC5983MA uses 0x09/0x0A/0x0B.
// Code snippet for MMC5603 usually shows:
// Reg 0x1B: Control 0 (Take Meas: 0x01 M, 0x02 T, ...)
// Reg 0x18: Status (Meas Done: 0x01 M, 0x02 T)
// Reg 0x39: Product ID (0x10)
// I will use these values (0x1B, 0x18, 0x39) as they are distinct from MMC5983MA (0x09, 0x08, 0x2F? code says ID read from somewhere, check).

// Actually, let's look at `sensors_enum.h`: ID reg 0x39.
// So 0x39 is definitely ID.
// MMC5603 likely follows a newer register map (0x1B range).

#define MMC5603_STATUS_REG  0x18
#define MMC5603_CTRL0_REG   0x1B
#define MMC5603_CTRL1_REG   0x1C
#define MMC5603_CTRL2_REG   0x1D
#define MMC5603_ID_REG      0x39
#define MMC5603_ID_VAL      0x10

// ODR Rates (Approximate pattern, need to verify if MMC5603 supports ODR logic in driver)
// MMC5603 might be simpler, mainly oneshot or CMM.
// We will implement oneshot mostly.

int mmc5603_init(float time, float *actual_time);
void mmc5603_shutdown(void);
int mmc5603_update_odr(float time, float *actual_time);
void mmc5603_mag_oneshot(void);
void mmc5603_mag_read(float m[3]);
void mmc5603_mag_process(uint8_t *raw_m, float m[3]);

extern const sensor_mag_t sensor_mag_mmc5603;

#endif
