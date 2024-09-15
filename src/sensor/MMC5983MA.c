/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432 Breakout Board.
  The MMC5983MA is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/
#include <math.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#include "MMC5983MA.h"

static const float sensitivity = (1.0f / 16384.0f); // mag sensitivity if using 18 bit data (16384 Counts/G)
static const float offset = 131072.0f; // mag range unsigned to signed

static uint8_t last_odr = 0xff;
static float last_time = 0;
static uint8_t last_rawTemp;
static float bridge_offset[3] = {0};

LOG_MODULE_REGISTER(MMC5983MA, LOG_LEVEL_DBG);

static void mmc_readData(const struct i2c_dt_spec *dev_i2c, uint32_t *data);
static void mmc_SET(const struct i2c_dt_spec *dev_i2c);
static void mmc_RESET(const struct i2c_dt_spec *dev_i2c);

int mmc_init(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time)
{
	// moved here, it is mmc specific
	mmc_SET(dev_i2c);													 // "deGauss" magnetometer

	// enable auto set/reset (bit 5 == 1)
	int err = i2c_reg_write_byte_dt(dev_i2c, MMC5983MA_CONTROL_0, 0x20);
	if (err)
		LOG_ERR("I2C error");

	last_odr = 0xff; // reset last odr
	err |= mmc_update_odr(dev_i2c, time, actual_time);
	return (err < 0 ? err : 0);
}

void mmc_shutdown(const struct i2c_dt_spec *dev_i2c)
{
	// reset device
	last_odr = 0xff; // reset last odr
	int err = i2c_reg_write_byte_dt(dev_i2c, MMC5983MA_CONTROL_1, 0x80); // Don't need to wait for MMC to finish reset
	if (err)
		LOG_ERR("I2C error");
}

int mmc_update_odr(const struct i2c_dt_spec *dev_i2c, float time, float *actual_time)
{
	int ODR;
	uint8_t MODR;
	uint8_t MBW;
	uint8_t MSET = MSET_2000; // always use lowest SET/RESET interval
	last_time = time;

	if (time <= 0 || time == INFINITY) // off interpreted as oneshot
		ODR = 0;
	else
		ODR = 1 / time;

	if (ODR > 200) // TODO: this sucks
	{ // 1000Hz*0.5ms/1000ms = 50% active
		MODR = MODR_1000Hz;
		MBW = MBW_800Hz; // Max MBW_800Hz
		time = 1.0 / 1000;
	}
	else if (ODR > 100) // Nominal working state, this should use as low power as possible
	{ // 200Hz*0.5ms/1000ms = 10% active
		MODR = MODR_200Hz;
		MBW = MBW_800Hz; // Max MBW_200Hz, MBW_800Hz is used since the RMS noise is still only 1.2mG
		time = 1.0 / 200;
	}
	else if (ODR > 50)
	{ // 100Hz*2ms/1000ms = 20% active
		MODR = MODR_100Hz;
		MBW = MBW_400Hz; // 0.8mG
		time = 1.0 / 100;
	}
	else if (ODR > 20)
	{ // 50Hz*4ms/1000ms = 20% active
		MODR = MODR_50Hz;
		MBW = MBW_200Hz; // 0.6mG
		time = 1.0 / 50;
	}
	else if (ODR > 10)
	{ // 20Hz*8ms/1000ms = 16% active
		MODR = MODR_20Hz;
		MBW = MBW_100Hz; // 0.4mG
		time = 1.0 / 20;
	}
	else if (ODR > 1)
	{ // 10Hz*8ms/1000ms = 8% active
		MODR = MODR_10Hz;
		MBW = MBW_100Hz;
		time = 1.0 / 10;
	}
	else if (ODR > 0)
	{ // 1Hz*8ms/1000ms = 0.8% active
		MODR = MODR_1Hz;
		MBW = MBW_100Hz;
		time = 1.0 / 1;
	}
	else
	{
		MODR = MODR_ONESHOT;
		MBW = MBW_800Hz;
		time = INFINITY;
	}

	if (last_odr == MODR)
		return -1;
	else
		last_odr = MODR;

	// set magnetometer bandwidth
	int err = i2c_reg_write_byte_dt(dev_i2c, MMC5983MA_CONTROL_1, MBW);

	// enable continuous measurement mode (bit 3 == 1), set sample rate
	// enable automatic Set/Reset (bit 8 == 1), set set/reset rate
	err |= i2c_reg_write_byte_dt(dev_i2c, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | 0x08 | MODR);
	if (err)
		LOG_ERR("I2C error");

	*actual_time = time;
	return err;
}

void mmc_mag_oneshot(const struct i2c_dt_spec *dev_i2c)
{
	// enable auto set/reset (bit 5 == 1) and trigger oneshot
	int err = i2c_reg_write_byte_dt(dev_i2c, MMC5983MA_CONTROL_0, 0x20 | 0x02);
	if (err)
		LOG_ERR("I2C error");
}

void mmc_mag_read(const struct i2c_dt_spec *dev_i2c, float m[3])
{
	int err = 0;
	uint8_t status;
	while (status & 0x02) // wait for oneshot to complete
		err |= i2c_reg_read_byte_dt(dev_i2c, MMC5983MA_CONTROL_0, &status);
	if (err)
		LOG_ERR("I2C error");
	uint32_t rawMag[3];
	mmc_readData(dev_i2c, rawMag);
	for (int i = 0; i < 3; i++) // x, y, z
		m[i] = ((float)rawMag[i] - offset) * sensitivity - bridge_offset[i];
}

// MMC must trigger the measurement, which will take significant time
// instead, the temperature is read from the last measurement and then another measurement is immediately triggered
float mmc_temp_read(const struct i2c_dt_spec *dev_i2c) // TODO: Not working
{
	uint8_t rawTemp;
	int err = i2c_reg_read_byte_dt(dev_i2c, MMC5983MA_TOUT, &rawTemp);
	// Temperature output, unsigned format. The range is -75~125°C, about 0.8°C/LSB, 00000000 stands for -75°C
	float temp = rawTemp;
	temp *= 0.8;
	temp -= 75;

	// USING SET AND RESET TO REMOVE BRIDGE OFFSET in datasheet
	if (last_rawTemp != rawTemp && last_odr > 1.0 / 50) // calculate offset at low motion only
	{ // TODO: does the temp register have hysteresis?
		float mPos[3], mNeg[3];
		float actual_time;

		last_rawTemp = rawTemp;
		for (int i = 0; i < 3; i++) // clear stored offset
			bridge_offset[i] = 0;

		err |= mmc_update_odr(dev_i2c, INFINITY, &actual_time); // set oneshot mode

		mmc_SET(dev_i2c);
		mmc_mag_oneshot(dev_i2c);
		mmc_mag_read(dev_i2c, mPos);

		mmc_RESET(dev_i2c);
		mmc_mag_oneshot(dev_i2c);
		mmc_mag_read(dev_i2c, mNeg);

		for (int i = 0; i < 3; i++) // store bridge offset for future readings
			bridge_offset[i] = (mPos[i] + mNeg[i]) / 2; // ((+H + Offset) + (-H + Offset)) / 2

		mmc_SET(dev_i2c);
		err |= mmc_update_odr(dev_i2c, last_time, &actual_time); // reset odr
	}

	// enable auto set/reset (bit 5 == 1) and trigger measurement
	err |= i2c_reg_write_byte_dt(dev_i2c, MMC5983MA_CONTROL_0, 0x20 | 0x01);
	if (err)
		LOG_ERR("I2C error");
	return temp;
}

static void mmc_readData(const struct i2c_dt_spec *dev_i2c, uint32_t *data)
{
	uint8_t rawData[7]; // x/y/z mag register data stored here
	int err = i2c_burst_read_dt(dev_i2c, MMC5983MA_XOUT_0, &rawData[0], 7); // Read the 7 raw data registers into data array
	if (err)
		LOG_ERR("I2C error");
	data[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value
	data[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // Turn the 18 bits into a unsigned 32-bit value
	data[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value
}

static void mmc_SET(const struct i2c_dt_spec *dev_i2c)
{
	int err = i2c_reg_write_byte_dt(dev_i2c, MMC5983MA_CONTROL_0, 0x08);
	if (err)
		LOG_ERR("I2C error");
	k_busy_wait(1); // self clearing after 500 ns
}

static void mmc_RESET(const struct i2c_dt_spec *dev_i2c)
{
	int err = i2c_reg_write_byte_dt(dev_i2c, MMC5983MA_CONTROL_0, 0x10);
	if (err)
		LOG_ERR("I2C error");
	k_busy_wait(1); // self clearing after 500 ns
}

const sensor_mag_t sensor_mag_mmc5983ma = {
	*mmc_init,
	*mmc_shutdown,

	*mmc_update_odr,

	*mmc_mag_oneshot,
	*mmc_mag_read,
	*mmc_temp_read
};
