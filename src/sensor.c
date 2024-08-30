#include "globals.h"
#include "sys.h"
#include "util.h"
#include "connection.h"

#include <math.h>

#include "fusion.h"
#include "vqf.h"
#include "magneto/magneto1_4.h"

#include "sensor/sensors.h"

#include "sensor.h"

#define SENSOR_IMU_NODE DT_NODELABEL(imu)
#define SENSOR_MAG_NODE DT_NODELABEL(mag)

struct i2c_dt_spec sensor_imu_dev = I2C_DT_SPEC_GET(SENSOR_IMU_NODE);
struct i2c_dt_spec sensor_mag_dev = I2C_DT_SPEC_GET(SENSOR_MAG_NODE);

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};			// vector to hold quaternion
float last_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};		// vector to hold quaternion

int64_t last_data_time;

float accelBias[3] = {0}, gyroBias[3] = {0}; // offset biases for the accel and gyro

int magCal;
int last_magCal;
int64_t magCal_time;
double ata[100]; // init cal
double norm_sum;
double sample_count;

float magBAinv[4][3];

float max_gyro_speed_square;
bool mag_use_oneshot;

float accel_actual_time;
float gyro_actual_time;
float mag_actual_time;

bool sensor_fusion_init;
bool sensor_sensor_init;

bool mag_available;
bool mag_enabled = MAG_ENABLED; // TODO: toggle from server

const sensor_fusion_t *sensor_fusion = &sensor_fusion_fusion; // TODO: change from server

const sensor_imu_t *sensor_imu = &sensor_imu_icm42688;
const sensor_mag_t *sensor_mag = &sensor_mag_mmc5983ma;

LOG_MODULE_REGISTER(sensor, LOG_LEVEL_INF);

K_THREAD_DEFINE(main_imu_thread_id, 4096, main_imu_thread, NULL, NULL, NULL, 7, 0, 0);

int sensor_init(void)
{
	if (sensor_sensor_init)
		return 0; // already initialized

	sys_read(); // In case ram not validated yet

	sensor_scan_read();

	LOG_INF("Scanning bus for IMU");
	int imu_id = sensor_scan_imu(&sensor_imu_dev); // TODO: the dev addr should be stored to retained and reused to save time
	if (imu_id >= (int)(sizeof(dev_imu_names) / sizeof(dev_imu_names[0])))
		LOG_WRN("Found unknown device");
	else if (imu_id < 0)
		LOG_ERR("No IMU detected");
	else
		LOG_INF("Found %s", dev_imu_names[imu_id]);
	if (imu_id >= 0)
	{
		if (imu_id >= (int)(sizeof(sensor_imus) / sizeof(sensor_imus[0])) || sensor_imus[imu_id] == NULL)
		{
			LOG_ERR("IMU not supported");
			sensor_imu = NULL;
			return -1; // an IMU was detected but not supported
		}
		else
		{
			sensor_imu = sensor_imus[imu_id];
		}
	}
	else
	{
		sensor_imu = NULL;
		return -1; // no IMU detected! something is very wrong
	}

	LOG_INF("Scanning bus for magnetometer");
	int mag_id = sensor_scan_mag(&sensor_mag_dev);
	if (mag_id >= (int)(sizeof(dev_mag_names) / sizeof(dev_mag_names[0])))
		LOG_WRN("Found unknown device");
	else if (mag_id < 0)
		LOG_WRN("No magnetometer detected");
	else
		LOG_INF("Found %s", dev_mag_names[mag_id]);
	if (mag_id >= 0) // if there is no magnetometer we do not care as much
	{
		if (mag_id >= (int)(sizeof(dev_mag_names) / sizeof(dev_mag_names[0])) || sensor_mags[mag_id] == NULL)
		{
			LOG_ERR("Magnetometer not supported");
			sensor_mag = NULL; 
			mag_available = false;
		}
		else
		{
			sensor_mag = sensor_mags[mag_id];
			mag_available = true;
		}
	}
	else
	{
		sensor_mag = NULL; 
		mag_available = false; // marked as not available
	}

	sensor_scan_write();

	sensor_sensor_init = true; // successfully initialized
	return 0;
}

void sensor_scan_read(void) // TODO: move some of this to sys?
{
	if (retained.imu_addr != 0)
		sensor_imu_dev.addr = retained.imu_addr;
	if (retained.mag_addr != 0)
		sensor_mag_dev.addr = retained.mag_addr;
	LOG_INF("IMU address: 0x%02x", sensor_imu_dev.addr);
	LOG_INF("Magnetometer address: 0x%02x", sensor_mag_dev.addr);
}

void sensor_scan_write(void) // TODO: move some of this to sys?
{
	retained.imu_addr = sensor_imu_dev.addr;
	retained.mag_addr = sensor_mag_dev.addr;
	retained_update();
}

void sensor_scan_clear(void) // TODO: move some of this to sys?
{
	retained.imu_addr = 0;
	retained.mag_addr = 0;
	retained_update();
}

void sensor_retained_read(void) // TODO: move some of this to sys?
{
	// Read calibration from retained
	memcpy(accelBias, retained.accelBias, sizeof(accelBias));
	memcpy(gyroBias, retained.gyroBias, sizeof(gyroBias));
	memcpy(magBAinv, retained.magBAinv, sizeof(magBAinv));
	LOG_INF("Main accelerometer bias: %.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
	LOG_INF("Main gyroscope bias: %.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);
	LOG_INF("Main magnetometer matrix:");
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);

	if (retained.fusion_data_stored)
	{
		LOG_INF("Fusion data recovered");
	}
}

// TODO: Always store quat(maybe) and just check if the fusion needs converge fast! (initialize)
// TODO: check if it is a vqf or fusion state
void sensor_retained_write(void) // TODO: move to sys?
{
	if (!sensor_fusion_init)
		return;
	(*sensor_fusion->save)(retained.fusion_data);
	retained.fusion_data_stored = true;
	retained_update();
}

void sensor_shutdown(void) // Communicate all imus to shut down
{
	int err = sensor_init(); // try initialization if possible
	if (!err)
		(*sensor_imu->shutdown)(&sensor_imu_dev);
	else
		LOG_ERR("Failed to shutdown sensor");
	if (mag_available)
		(*sensor_mag->shutdown)(&sensor_mag_dev);
};

void sensor_setup_WOM(void)
{
	int err = sensor_init(); // try initialization if possible
	if (!err)
		(*sensor_imu->setup_WOM)(&sensor_imu_dev);
	else
		LOG_ERR("Failed to setup wake on motion");
}

void sensor_calibrate_imu(void)
{
	LOG_INF("Calibrating main accelerometer and gyroscope zero rate offset");
	// TODO: Add LED flashies
	LOG_INF("Rest the device on a stable surface");
	if (!wait_for_motion(&sensor_imu_dev, false, 20)) // Wait for accelerometer to settle, timeout 10s
		return; // Timeout, calibration failed

	set_led(SYS_LED_PATTERN_ON); // scuffed led
	k_msleep(500); // Delay before beginning acquisition

	LOG_INF("Reading data");
	sensor_offsetBias(&sensor_imu_dev, accelBias, gyroBias); // This takes about 3s
	sys_write(MAIN_ACCEL_BIAS_ID, &retained.accelBias, accelBias, sizeof(accelBias));
	sys_write(MAIN_GYRO_BIAS_ID, &retained.gyroBias, gyroBias, sizeof(gyroBias));
	LOG_INF("%.5f %.5f %.5f", accelBias[0], accelBias[1], accelBias[2]);
	LOG_INF("%.5f %.5f %.5f", gyroBias[0], gyroBias[1], gyroBias[2]);

	LOG_INF("Finished calibration");
	if (sensor_fusion_init)
	{ // clear fusion gyro offset
		float g_off[3] = {0};
		(*sensor_fusion->set_gyro_bias)(g_off);
		sensor_retained_write();
	}
	else
	{
		retained.fusion_data_stored = false; // Invalidate retained fusion data
	}
	set_led(SYS_LED_PATTERN_OFF); // scuffed led
}

void sensor_calibrate_mag(void)
{
	LOG_INF("Calibrating magnetometer hard/soft iron offset");
	magneto_current_calibration(magBAinv, ata, norm_sum, sample_count); // 25ms
	sys_write(MAIN_MAG_BIAS_ID, &retained.magBAinv, magBAinv, sizeof(magBAinv));
	for (int i = 0; i < 3; i++)
		LOG_INF("%.5f %.5f %.5f %.5f", magBAinv[0][i], magBAinv[1][i], magBAinv[2][i], magBAinv[3][i]);
	LOG_INF("Finished calibration");
	//magCal |= 1 << 7;
	magCal = 0;
	// clear data
	//memset(ata[0], 0, sizeof(ata)); // does this work??
	for (int i = 0; i < 100; i++)
		ata[i] = 0.0;
	norm_sum = 0.0;
	sample_count = 0.0;
}

// TODO: get rid of it
void set_LN(void)
{
	tickrate = 6;
}

// TODO: get rid of it
void set_LP(void)
{
	tickrate = 33;
}

bool wait_for_motion(const struct i2c_dt_spec *dev_i2c, bool motion, int samples)
{
	uint8_t counts = 0;
	float a[3], last_a[3];
	(*sensor_imu->accel_read)(dev_i2c, last_a);
	set_led(SYS_LED_PATTERN_LONG);
	for (int i = 0; i < samples + counts; i++)
	{
		LOG_INF("Accelerometer: %.5f %.5f %.5f", a[0], a[1], a[2]);
		k_msleep(500);
		(*sensor_imu->accel_read)(dev_i2c, a);
		if (v_epsilon(a, last_a, 0.1) != motion)
		{
			LOG_INF("No motion detected");
			counts++;
			if (counts == 2)
			{
				set_led(SYS_LED_PATTERN_OFF);
				return true;
			}
		}
		else
		{
			counts = 0;
		}
		memcpy(last_a, a, sizeof(a));
	}
	LOG_INF("Motion detected");
	set_led(SYS_LED_PATTERN_OFF);
	return false;
}

int main_imu_init(void)
{
	int err;
	// TODO: on any errors set main_ok false and skip (make functions return nonzero)
// 5ms delta (???) from entering loop
// skip sleep, surely this wont cause issues :D
/*
	int64_t time_delta = k_uptime_get() - start_time;
	if (time_delta < 11)
		k_msleep(11 - time_delta);
	//k_msleep(11);														 // Wait for start up (1ms for ICM, 10ms for MMC -> 10ms)
*/
	err = sensor_init(); // IMUs discovery
	if (err)
		return err;
	LOG_INF("Found main IMUs");
	// TODO: This may change!!
	//i2c_reg_write_byte_dt(&main_imu, ICM42688_DEVICE_CONFIG, 0x01); // i dont wanna wait on icm!!
	(*sensor_imu->shutdown)(&sensor_imu_dev);
	// TODO: This may change!!
	//i2c_reg_write_byte_dt(&main_mag, MMC5983MA_CONTROL_1, 0x80); // Reset MMC now to avoid waiting 10ms later
	if (mag_available)
		(*sensor_mag->shutdown)(&sensor_mag_dev);
	//icm_reset(main_imu);												 // software reset ICM42688 to default registers
	// TODO: Does int flag need to be read at all
	//uint8_t temp;
	//i2c_reg_read_byte_dt(&main_imu, ICM42688_INT_STATUS, &temp); // clear reset done int flag

	float clock_actual_rate;
	set_sensor_clock(true, 32768, &clock_actual_rate); // enable the clock source for IMU if present

	err = (*sensor_imu->init)(&sensor_imu_dev, clock_actual_rate, tickrate / 1000.0, 1.0 / 800, &accel_actual_time, &gyro_actual_time); // configure with ~200Hz ODR, ~1000Hz ODR
	if (err < 0)
		return err;
// 55-66ms delta to wait, get chip ids, and setup icm (50ms spent waiting for accel and gyro to start)
	if (mag_available && mag_enabled)
	{
		err = (*sensor_mag->init)(&sensor_mag_dev, tickrate / 1000.0, &mag_actual_time);								 // configure with ~200Hz ODR
		if (err < 0)
			return err;
// 0-1ms delta to setup mmc
	}
	LOG_INF("Initialized main IMUs");

	sys_read(); // In case calibrations haven't loaded yet

	do
	{
		if (reset_mode == 1) // Reset mode main calibration
		{
			sensor_calibrate_imu();
			reset_mode = 0; // Clear reset mode
		}
	} while (false); // TODO: ????? why is this here

	// Setup fusion
	LOG_INF("Initialize fusion");
	sensor_retained_read();
	(*sensor_fusion->init)(gyro_actual_time);
	if (retained.fusion_data_stored)
	{ // Load state if the data is valid (fusion was initialized before)
		(*sensor_fusion->load)(retained.fusion_data);
		retained.fusion_data_stored = false; // Invalidate retained fusion data
		retained_update();
	}

	LOG_INF("Initialized fusion");
	sensor_fusion_init = true;
	return 0;
}

bool reconfig;
int powerstate = 0;
int last_powerstate = 0;

void main_imu_thread(void)
{
	main_running = true;
	int err = main_imu_init(); // Initialize IMUs and Fusion
	// TODO: handle imu init error
	if (!err)
		main_ok = true;
	while (1)
	{
		if (main_ok)
		{
			// Trigger reconfig on powerstate change
			reconfig = last_powerstate != powerstate ? true : false;
			last_powerstate = powerstate;

			// Reading IMUs will take between 2.5ms (~7 samples, low noise) - 7ms (~33 samples, low power)
			// Magneto sample will take ~400us
			// Fusing data will take between 100us (~7 samples, low noise) - 500us (~33 samples, low power)
			// TODO: on any errors set main_ok false and skip (make functions return nonzero)

			// At high speed, use oneshot mode to have synced magnetometer data
			// Call before FIFO and get the data after
			if (mag_available && mag_enabled && mag_use_oneshot)
				(*sensor_mag->mag_oneshot)(&sensor_mag_dev);

			// Read gyroscope (FIFO)
			uint8_t rawData[2080];
			uint16_t packets = (*sensor_imu->fifo_read)(&sensor_imu_dev, rawData); // TODO: name this better?
			LOG_DBG("IMU packet count: %u", packets);

			// Read accelerometer
			float raw_a[3];
			(*sensor_imu->accel_read)(&sensor_imu_dev, raw_a);
			float ax = raw_a[0] - accelBias[0];
			float ay = raw_a[1] - accelBias[1];
			float az = raw_a[2] - accelBias[2];

			// Read magnetometer and process magneto
			float mx = 0, my = 0, mz = 0;
			if (mag_available && mag_enabled && powerstate == 0)
			{
				float m[3];
				(*sensor_mag->mag_read)(&sensor_mag_dev, m);
				magneto_sample(m[0], m[1], m[2], ata, &norm_sum, &sample_count); // 400us
				apply_BAinv(m, magBAinv);
				mx = m[0];
				my = m[1];
				mz = m[2];
				int new_magCal = magCal;
				new_magCal |= (-1.2 < ax && ax < -0.8 ? 1 << 0 : 0) | (1.2 > ax && ax > 0.8 ? 1 << 1 : 0) | // dumb check if all accel axes were reached for cal, assume the user is intentionally doing this
					(-1.2 < ay && ay < -0.8 ? 1 << 2 : 0) | (1.2 > ay && ay > 0.8 ? 1 << 3 : 0) |
					(-1.2 < az && az < -0.8 ? 1 << 4 : 0) | (1.2 > az && az > 0.8 ? 1 << 5 : 0);
				if (new_magCal > magCal && new_magCal == last_magCal)
				{
					if (k_uptime_get() > magCal_time)
					{
						magCal = new_magCal;
						LOG_INF("Magnetometer calibration progress: %d", new_magCal);
					}
				}
				else
				{
					magCal_time = k_uptime_get() + 1000;
					last_magCal = new_magCal;
				}
				if (magCal == 0b111111)
					set_led(SYS_LED_PATTERN_ON); // TODO: will interfere with things
			}

			if (mag_available && mag_enabled && reconfig) // TODO: get rid of reconfig?
			{
				switch (powerstate)
				{
				case 0:
					set_LN();
					LOG_INF("Switching main IMUs to low noise");
					break;
				case 1:
					set_LP();
					LOG_INF("Switching main IMUs to low power");
					(*sensor_mag->update_odr)(&sensor_mag_dev, INFINITY, &mag_actual_time); // standby/oneshot
					break;
				};
			}

			float a[] = {ax, -az, ay};
			if (mag_available && mag_enabled && packets == 2 && powerstate == 1) // why specifically 2 packets? i forgot
			{ // Fuse accelerometer only
				(*sensor_fusion->update_accel)(a, accel_actual_time);
			}
			else
			{ // Fuse all data
				float g[3] = {0};
				float m[] = {my, mz, -mx};
				max_gyro_speed_square = 0;
				for (uint16_t i = 0; i < packets; i++)
				{
					float raw_g[3];
					if ((*sensor_imu->fifo_process)(i, rawData, raw_g))
						continue; // skip on error
					// transform and convert to float values
					float gx = raw_g[0] - gyroBias[0]; //gres
					float gy = raw_g[1] - gyroBias[1]; //gres
					float gz = raw_g[2] - gyroBias[2]; //gres
					//float g[] = {gx, -gz, gy};
					g[0] = gx;
					g[1] = -gz;
					g[2] = gy;

					// Process fusion
					(*sensor_fusion->update)(g, a, m, gyro_actual_time);

					if (mag_available && mag_enabled)
					{
						// Get fusion's corrected gyro data (or get gyro bias from fusion) and use it here
						float g_off[3] = {};
						(*sensor_fusion->get_gyro_bias)(g_off);
						for (int i = 0; i < 3; i++)
						{
							g_off[i] = g[i] - g_off[i];
						}

						// Get the highest gyro speed
						float gyro_speed_square = g_off[0] * g_off[0] + g_off[1] * g_off[1] + g_off[2] * g_off[2];
						if (gyro_speed_square > max_gyro_speed_square)
							max_gyro_speed_square = gyro_speed_square;
					}
				}

				// Update fusion gyro sanity?
				(*sensor_fusion->update_gyro_sanity)(g, m);
			}

			// Get updated linear acceleration and quaternion from fusion
			float lin_a[3] = {0};
			(*sensor_fusion->get_lin_a)(lin_a);
			(*sensor_fusion->get_quat)(q);
			q_normalize(q, q); // safe to use self as output

			// Check the IMU gyroscope
			if ((*sensor_fusion->get_gyro_sanity)() == 0 ? q_epsilon(q, last_q, 0.001) : q_epsilon(q, last_q, 0.01)) // Probably okay to use the constantly updating last_q
			{
				int64_t imu_timeout = CLAMP(last_data_time, 1 * 1000, 15 * 1000); // Ramp timeout from last_data_time
				if (k_uptime_get() - last_data_time > imu_timeout) // No motion in last 1s - 10s
				{
					LOG_INF("No motion from main IMUs in %llds", imu_timeout/1000);
					system_off_main = true;
				}
				else if (powerstate == 0 && k_uptime_get() - last_data_time > 500) // No motion in last 500ms
				{
					LOG_INF("No motion from main IMUs in 500ms");
					powerstate = 1;
				}
			}
			else
			{
				last_data_time = k_uptime_get();
				powerstate = 0;
			}

			// Update magnetometer mode
			if (mag_available && mag_enabled && powerstate == 0)
			{
				float gyro_speed = sqrtf(max_gyro_speed_square);
				float mag_target_time = 1.0 / (4 * gyro_speed); // target mag ODR for ~0.25 deg error
				if (mag_target_time > 0.005) // cap at 0.005 (200hz), above this the sensor will use oneshot mode instead
				{
					mag_target_time = 0.005;
					int err = (*sensor_mag->update_odr)(&sensor_mag_dev, INFINITY, &mag_actual_time);
					if (!err)
						LOG_DBG("Switching magnetometer to oneshot");
					mag_use_oneshot = true;
				}
				if (mag_target_time <= 0.005 || mag_actual_time != INFINITY) // under 200Hz or magnetometer did not have a oneshot mode
				{
					int err = (*sensor_mag->update_odr)(&sensor_mag_dev, mag_target_time, &mag_actual_time);
					if (!err)
						LOG_DBG("Switching magnetometer ODR to %.2fHz", 1.0 / mag_actual_time);
					mag_use_oneshot = false;
				}
			}

			// Send packet with new orientation
			if (!(q_epsilon(q, last_q, 0.0002)))
			{
				memcpy(last_q, q, sizeof(q));
				float q_offset[4];
				q_multiply(q, q3, q_offset);
				connection_write_packet_0(q_offset, lin_a);
			}

			// Handle magnetometer calibration or bridge offset calibration
			if (mag_available && mag_enabled && last_powerstate == 1 && powerstate == 1) // TODO: i guess this is fine
			{
				if (magCal == 0b111111) // Save magCal while idling
					sensor_calibrate_mag();
				else // only enough time to do one of the two
					(*sensor_mag->temp_read)(&sensor_mag_dev); // for some applicable magnetometer, calibrates bridge offsets
			}
		}
		main_running = false;
		k_sleep(K_FOREVER);
		main_running = true;
	}
}

void wait_for_threads(void)
{
	if (threads_running || main_running)
		while (main_running)
			k_msleep(1); // bane of my existence. don't use k_yield()!!!!!!
//			k_usleep(1);
}

void main_imu_suspend(void)
{
	k_thread_suspend(main_imu_thread_id);
	main_running = false;
	LOG_INF("Suspended main IMU thread");
}

void main_imu_wakeup(void)
{
	k_wakeup(main_imu_thread_id);
}

// TODO: move to a calibration file
// TODO: setup 6 sided calibration (bias and scale, and maybe gyro ZRO?), setup temp calibration (particulary for gyro ZRO)
void sensor_offsetBias(const struct i2c_dt_spec *dev_i2c, float *dest1, float *dest2)
{
	float rawData[3];
	for (int i = 0; i < 500; i++)
	{
		(*sensor_imu->accel_read)(dev_i2c, &rawData[0]);
		dest1[0] += rawData[0];
		dest1[1] += rawData[1];
		dest1[2] += rawData[2];
		(*sensor_imu->gyro_read)(dev_i2c, &rawData[0]);
		dest2[0] += rawData[0];
		dest2[1] += rawData[1];
		dest2[2] += rawData[2];
		k_msleep(5);
	}

	dest1[0] /= 500.0f;
	dest1[1] /= 500.0f;
	dest1[2] /= 500.0f;
	dest2[0] /= 500.0f;
	dest2[1] /= 500.0f;
	dest2[2] /= 500.0f;
// need better accel calibration
	if(dest1[0] > 0.9f) {dest1[0] -= 1.0f;} // Remove gravity from the x-axis accelerometer bias calculation
	if(dest1[0] < -0.9f) {dest1[0] += 1.0f;} // Remove gravity from the x-axis accelerometer bias calculation
	if(dest1[1] > 0.9f) {dest1[1] -= 1.0f;} // Remove gravity from the y-axis accelerometer bias calculation
	if(dest1[1] < -0.9f) {dest1[1] += 1.0f;} // Remove gravity from the y-axis accelerometer bias calculation
	if(dest1[2] > 0.9f) {dest1[2] -= 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
	if(dest1[2] < -0.9f) {dest1[2] += 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
}
