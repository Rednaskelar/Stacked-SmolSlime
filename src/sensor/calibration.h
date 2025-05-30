/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#ifndef SLIMENRF_SENSOR_CALIBRATION
#define SLIMENRF_SENSOR_CALIBRATION

/* Sensor feeds data to calibration */
void sensor_calibration_process_accel(float a[3]);
void sensor_calibration_process_gyro(float g[3]);
void sensor_calibration_process_mag(float m[3]);

void sensor_calibration_update_sensor_ids(int imu);
uint8_t *sensor_calibration_get_sensor_data();

void sensor_calibration_read(void);

int sensor_calibration_validate(bool write);
int sensor_calibration_validate_6_side(bool write);
int sensor_calibration_validate_mag(bool write);

void sensor_calibration_clear(bool write);
void sensor_calibration_clear_6_side(bool write);
void sensor_calibration_clear_mag(bool write); // "request" mag cal

void sensor_request_calibration(void);
void sensor_request_calibration_6_side(void);
void sensor_request_calibration_mag(void);

#endif