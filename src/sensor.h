#ifndef SLIMENRF_SENSOR
#define SLIMENRF_SENSOR

void sensor_retained_read(void);
void sensor_retained_write(void);

void sensor_shutdown(void);
void sensor_setup_WOM(void);

void sensor_calibrate_imu(void);
void sensor_calibrate_mag(void);

void set_LN(void);
void set_LP(void);
bool wait_for_motion(const struct i2c_dt_spec imu, bool motion, int samples);

// TODO: make threads more abstract, pass in imus n stuff instead
void main_imu_init(void);
void main_imu_thread(void);
void wait_for_threads(void);
void main_imu_suspend(void);
void main_imu_wakeup(void);

void sensor_offsetBias(struct i2c_dt_spec dev_i2c, float * dest1, float * dest2);

typedef struct sensor_fusion {
	void (*init)(float);
	void (*load)(const void *);
	void (*save)(void *);

	void (*update_accel)(float *, float);
	void (*update)(float *, float *, float *, float);

	void (*get_gyro_bias)(float *);
	void (*set_gyro_bias)(float *);

	void (*update_gyro_sanity)(float *, float *);
	int (*get_gyro_sanity)(void);

	void (*get_lin_a)(float *);
	void (*get_quat)(float *);
} sensor_fusion_t;

typedef struct sensor_imu {
	int (*init)(struct i2c_dt_spec, float, float, float*, float*); // return update time, return 0 if success, 1 if general error, also how to deal with CLKIN???? some boards might use it
	void (*shutdown)(struct i2c_dt_spec);

	int (*update_odr)(struct i2c_dt_spec, float, float, float*, float*); // return actual update time, return 0 if success, -1 if odr is same, 1 if general error

	uint16_t (*fifo_read)(struct i2c_dt_spec, uint8_t*);
	int (*fifo_process)(uint16_t, uint8_t*, float[3]); // deg/s TODO: is support accel needed?
	void (*accel_read)(struct i2c_dt_spec, float[3]); // m/s^2
	void (*gyro_read)(struct i2c_dt_spec, float[3]); // deg/s
	float (*temp_read)(struct i2c_dt_spec); // deg C

	void (*setup_WOM)(struct i2c_dt_spec);
} sensor_imu_t;

typedef struct sensor_mag {
	int (*init)(struct i2c_dt_spec, float, float*); // return update time, return 0 if success, 1 if general error
	void (*shutdown)(struct i2c_dt_spec);

	int (*update_odr)(struct i2c_dt_spec, float, float*); // return actual update time, return 0 if success, -1 if odr is same, 1 if general error

	void (*mag_oneshot)(struct i2c_dt_spec); // trigger oneshot if exists
	void (*mag_read)(struct i2c_dt_spec, float[3]); // any unit
} sensor_mag_t;

#endif