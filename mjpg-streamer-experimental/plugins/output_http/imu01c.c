/*  
    C interface for controlling a Pololu MinIMU-9 v3 sensor device (imu01c) plugged into a Raspberry Pi.
    Class built upon SMBus and i2C standard APIs.

    Works with sensor MinIMU-9 v3 (dicontinued) (https://www.pololu.com/product/2468)
    MinIMU-9 v3 Specsheet Reference https://www.pololu.com/file/0J703/LSM303D.pdf / https://www.pololu.com/file/0J731/L3GD20H.pdf

    Stand-alone compilation
    -----------------------
    libi2c.so.0.1.0/0.1.1 must exist on the system.
    See build_imu.sh for building details.

    Copyright (C) 2021 Captain Thunk

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include "smbus.h"
#include "imu01c.h"

void imu01c_update(imu01cClass *self) {
	self->setup(self);
	self->getMag(self);
	self->getAccel(self);
	self->getGyro(self);
	self->getLSM303DTemperature(self);
	self->getL3GD20HTemperature(self);
	self->getHeading(self);
	self->getTiltHeading(self);
}

// Write one byte to the i2c device
bool imu01c_i2c_write_byte(imu01cClass *self, int addr, int reg, unsigned char c)
{
	if (!self->check_int_range(c, 0, 0xFF)) return false;
	
	for (int i = 0; i < self->_i2c_retries; i++)
	{
		if (i2c_smbus_write_byte_data(addr, reg, c) >= 0) return true;
		self->lsleep(self->_i2c_retry_time);
	}
	fprintf(stderr, "%s : %d : %s : failed to write byte to the i2c device. \n", __FILE__, __LINE__, __func__);
	return false;
}

// Read one byte from the i2c device
bool imu01c_i2c_read_byte(imu01cClass *self, int addr, int reg, unsigned char * c)
{
	int res = -1;
	for (int i = 0; i < self->_i2c_retries; i++)
	{
		if ((res = i2c_smbus_read_byte_data(addr, reg)) >= 0)
		{
			*c = (res & 0xFF);
			return true;
		}
		self->lsleep(self->_i2c_retry_time);
	}
	fprintf(stderr, "%s : %d : %s : failed to read byte to the i2c device. \n", __FILE__, __LINE__, __func__);
	return false;
}
	
// Check bounds of an int value.
bool imu01c_check_int_range(int value, int value_min, int value_max)
{
	return ((value >= value_min) && (value <= value_max));
}

// Suspend this thread for ms milliseconds.
int imu01c_lsleep(long int ms)
{
	int result = 0;
	struct timespec ts_sleep, ts_remaining;
	ts_remaining.tv_sec = (time_t)(ms / 1000);
	ts_remaining.tv_nsec = (long)(ms % 1000) * 1000000;
	
	do
	{
		ts_sleep = ts_remaining;
		result = nanosleep(&ts_sleep, &ts_remaining);
	}
	
	while ((errno == EINTR) && (result == -1));
	
	if (result == -1)
	{
		fprintf(stderr, "%s : %d : %s nanosleep() failed. \n", __FILE__, __LINE__, __func__);
	}
	
	return result;
}

int imu01c_twos_comp(int val, int bits)
{
	// # Calculate the 2s complement of int:val #
	if ((val&(1 << (bits-1))) != 0)
	{
		val = val - (1 << bits);
	}

	return val;
}

void imu01c_getAccel(imu01cClass *self) {
	unsigned char aHi;
	unsigned char aLo;
	
	// x
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_X_H_A, &aHi))
		fprintf(stderr, "%s : %d : %s : unable to get accelerometer x hi byte. \n", __FILE__, __LINE__, __func__);
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_X_L_A, &aLo))
		fprintf(stderr, "%s : %d : %s : unable to get accelerometer x lo byte. \n", __FILE__, __LINE__, __func__);	
	self->accel_xyz[X] = self->twos_comp((aHi << 8 | aLo), 16)  / pow(2, 15) * LSM303D_ACCEL_SCALE;
	
	// y
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Y_H_A, &aHi))
		fprintf(stderr, "%s : %d : %s : unable to get accelerometer y hi byte. \n", __FILE__, __LINE__, __func__);
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Y_L_A, &aLo))
		fprintf(stderr, "%s : %d : %s : unable to get accelerometer y lo byte. \n", __FILE__, __LINE__, __func__);
	self->accel_xyz[Y] = self->twos_comp((aHi << 8 | aLo), 16)  / pow(2, 15) * LSM303D_ACCEL_SCALE;

	// z
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Z_H_A, &aHi))
		fprintf(stderr, "%s : %d : %s : unable to get accelerometer z hi byte. \n", __FILE__, __LINE__, __func__); 
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Z_L_A, &aLo))
		fprintf(stderr, "%s : %d : %s : unable to get accelerometer z lo byte. \n", __FILE__, __LINE__, __func__);
	self->accel_xyz[Z] = self->twos_comp((aHi << 8 | aLo), 16)  / pow(2, 15) * LSM303D_ACCEL_SCALE;

	// printf("X: %d, Y: %d, Z: %d \r\n", self->accel_xyz[0], self->accel_xyz[1], self->accel_xyz[2]);
	
}
	
void imu01c_getMag(imu01cClass *self) {
	unsigned char mHi;
	unsigned char mLo;
	
	// x
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_X_H_M, &mHi))
		fprintf(stderr, "%s : %d : %s : unable to get magnetometer x hi byte. \n", __FILE__, __LINE__, __func__);
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_X_L_M, &mLo))
		fprintf(stderr, "%s : %d : %s : unable to get magnetometer x lo byte. \n", __FILE__, __LINE__, __func__);
	self->mag_xyz[X] = self->twos_comp((mHi << 8 | mLo), 16);
	
	// y
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Y_H_M, &mHi))
		fprintf(stderr, "%s : %d : %s : unable to get magnetometer y hi byte. \n", __FILE__, __LINE__, __func__);
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Y_L_M, &mLo))
		fprintf(stderr, "%s : %d : %s : unable to get magnetometer y lo byte. \n", __FILE__, __LINE__, __func__);
	self->mag_xyz[Y] = self->twos_comp((mHi << 8 | mLo), 16);
	
	// z
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Z_H_M, &mHi))
		fprintf(stderr, "%s : %d : %s : unable to get magnetometer z hi byte. \n", __FILE__, __LINE__, __func__);
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_OUT_Z_L_M, &mLo))
		fprintf(stderr, "%s : %d : %s : unable to get magnetometer z lo byte. \n", __FILE__, __LINE__, __func__);
	self->mag_xyz[Z] = self->twos_comp((mHi << 8 | mLo), 16);
	
	// printf("X: %d, Y: %d, Z: %d \r\n", self->mag_xyz[0], self->mag_xyz[1], self->mag_xyz[2]); 				  
}

void imu01c_getGyro(imu01cClass *self) {
	unsigned char gHi;
	unsigned char gLo;
	
	// x
	if (!self->i2c_read_byte(self, self->i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_X_H, &gHi))
		fprintf(stderr, "%s : %d : %s : unable to get gyroscope x hi byte. \n", __FILE__, __LINE__, __func__);
	if (!self->i2c_read_byte(self, self->i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_X_L, &gLo))
		fprintf(stderr, "%s : %d : %s : unable to get gyroscope x lo byte. \n", __FILE__, __LINE__, __func__);
	self->gyro_xyz[X] = self->twos_comp((gHi << 8 | gLo), 16);
	
	// y
	if (!self->i2c_read_byte(self, self->i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_Y_H, &gHi))
		fprintf(stderr, "%s : %d : %s : unable to get gyroscope y hi byte. \n", __FILE__, __LINE__, __func__);
	if (!self->i2c_read_byte(self, self->i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_Y_L, &gLo))
		fprintf(stderr, "%s : %d : %s : unable to get gyroscope y lo byte. \n", __FILE__, __LINE__, __func__);
	self->gyro_xyz[Y] = self->twos_comp((gHi << 8 | gLo), 16);
	
	// z
	if (!self->i2c_read_byte(self, self->i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_Z_H, &gHi))
		fprintf(stderr, "%s : %d : %s : unable to get gyroscope z hi byte. \n", __FILE__, __LINE__, __func__);
	if (!self->i2c_read_byte(self, self->i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_Z_L, &gLo))
		fprintf(stderr, "%s : %d : %s : unable to get gyroscope z lo byte. \n", __FILE__, __LINE__, __func__);
	self->gyro_xyz[Z] = self->twos_comp((gHi << 8 | gLo), 16);
	
	// printf("X: %d, Y: %d, Z: %d \r\n", self->gyro_xyz[0], self->gyro_xyz[1], self->gyro_xyz[2]); 				  
}

void imu01c_getLSM303DTemperature(imu01cClass *self) {
	unsigned char tHi;
	unsigned char tLo;
	
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_TEMP_OUT_H, &tHi))
		fprintf(stderr, "%s : %d : %s : unable to get temperature hi byte. \n", __FILE__, __LINE__, __func__); 
	if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_TEMP_OUT_L, &tLo))
		fprintf(stderr, "%s : %d : %s : unable to get temperature lo byte. \n", __FILE__, __LINE__, __func__);
	
	self->LSM303D_Temperature = self->twos_comp(((tHi << 8) | tLo) >> 4, 12);
	
	self->LSM303D_TemperatureDegrees = (self->LSM303D_Temperature / 8.0) + TEMPERATURE_CELSIUS_OFFSET;
}

void imu01c_getL3GD20HTemperature(imu01cClass *self) {
	unsigned char byte;
	
	if (!self->i2c_read_byte(self, self->i2c_l3gd20h_addr, L3GD20H_REGISTER_OUT_TEMP, &byte))
		fprintf(stderr, "%s : %d : %s : unable to get temperature byte. \n", __FILE__, __LINE__, __func__);

	self->L3GD20H_Temperature = self->twos_comp(byte, 8);
	
	self->L3GD20H_TemperatureDegrees = (self->L3GD20H_Temperature / 8.0) + TEMPERATURE_CELSIUS_OFFSET;
}

void imu01c_getHeading(imu01cClass *self) { 
	/*
	self->heading = 180 * atan2(self->mag_xyz[Y], self->mag_xyz[X]) / M_PI;
	if(self->heading < 0) {
		self->heading += 360;
	}
	self->headingDegrees = self->heading;

	double mtMagX = self->mag_xyz[X] / 450 * 100.0;
	double mtMagY = self->mag_xyz[Y] / 450 * 100.0;
	// double mtMagZ = mag_xyz[Z] / 400 * 100.0;	// unused

	self->heading = atan2(mtMagX, mtMagY);
	

	/* original code - it's 90 deg out */
	self->heading = atan2(self->mag_xyz[X], self->mag_xyz[Y]);

	if(self->heading < 0) {
		self->heading += 2*M_PI;
	}
	if(self->heading > 2*M_PI) {
		self->heading -= 2*M_PI;
	}

	self->headingDegrees = self->heading * (180.0 / M_PI); 		// radians to degrees
}

bool imu01c_isMagReady(imu01cClass *self) { 
	unsigned char byte;
	
	for (int i = 0; i < self->_i2c_retries; i++)
	{
		if (!self->i2c_read_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_STATUS_M, &byte))
			fprintf(stderr, "%s : %d : %s : unable to get magnetometer status. \n", __FILE__, __LINE__, __func__);
		
		if (byte & 0x08)
		{
			return true;
		}
		self->lsleep(self->_i2c_retry_time);
	}
	
	return false;
}

void imu01c_getTiltHeading(imu01cClass *self) {
	double truncate[3] = {0, 0, 0};
	double tiltcomp[3] = {0, 0, 0};
	double pitch, roll;
	
	truncate[X] = copysign(min(fabs(self->accel_xyz[X]), 1.0), self->accel_xyz[X]);
	truncate[Y] = copysign(min(fabs(self->accel_xyz[Y]), 1.0), self->accel_xyz[Y]);
	truncate[Z] = copysign(min(fabs(self->accel_xyz[Z]), 1.0), self->accel_xyz[Z]);
	// truncate[X] = copysign(min(fabs(self->gyro_xyz[X]), 1.0), self->gyro_xyz[X]);
	// truncate[Y] = copysign(min(fabs(self->gyro_xyz[Y]), 1.0), self->gyro_xyz[Y]);
	// truncate[Z] = copysign(min(fabs(self->gyro_xyz[Z]), 1.0), self->gyro_xyz[Z]);
	
	pitch = asin(-1*truncate[X]);
	
	// roll = math.asin(truncate[Y]/math.cos(pitch)) if abs(math.cos(pitch)) >= abs(truncate[Y]) else 0
	
	if(abs(cos(pitch)) >= abs(truncate[Y])) {
		roll = asin(truncate[Y]/cos(pitch));
	} else {
		roll = 0.0;
	}
	// printf("Roll: %.4f \r\n",roll);
	// set roll to zero if pitch approaches -1 or 1

	tiltcomp[X] = self->mag_xyz[X] * cos(pitch) + self->mag_xyz[Z] * sin(pitch);
	tiltcomp[Y] = self->mag_xyz[X] * sin(roll) * sin(pitch) + self->mag_xyz[Y] * cos(roll) - self->mag_xyz[Z] * sin(roll) * cos(pitch);
	tiltcomp[Z] = self->mag_xyz[X] * cos(roll) * sin(pitch) + self->mag_xyz[Y] * sin(roll) + self->mag_xyz[Z] * cos(roll) * cos(pitch);
	self->tiltHeading = atan2(tiltcomp[Y], tiltcomp[X]);
/*
	if(self->tiltHeading < 0) {
		self->tiltHeading += 2*M_PI;
	}
	if(self->tiltHeading > (2*M_PI)) {
		self->heading -= 2*M_PI;
		// self->tiltHeading -= 2*M_PI;
	}
/**/
	self->tiltHeadingDegrees = self->tiltHeading * (180.0 / M_PI);

	self->tiltHeadingDegrees += TILT_HEADING_OFFSET;
	if(self->tiltHeadingDegrees < 0) {
		self->tiltHeadingDegrees += 360;
	}
	if(self->tiltHeadingDegrees >= 360) {
		self->tiltHeadingDegrees -= 360;
	}
}

bool imu01c_setup(imu01cClass *self)
{
	if (self->is_setup) return true;
	
	// set variables
	self->_i2c_retries   				= 10;
    self->_i2c_retry_time	 			= 50;
	self->i2c_lsm303d_addr 				= 0;
	self->i2c_l3gd20h_addr				= 0;
	self->accel_xyz[X]					= 0.0;
	self->accel_xyz[Y] 					= 0.0;
	self->accel_xyz[Z] 					= 0.0;
	self->mag_xyz[X] 					= 0;
	self->mag_xyz[Y] 					= 0;
	self->mag_xyz[Z] 					= 0;
	self->gyro_xyz[X] 					= 0;
	self->gyro_xyz[Y] 					= 0;
	self->gyro_xyz[Z] 					= 0;
	self->heading 						= 0;
	self->headingDegrees 				= 0;
	self->tiltHeading 					= 0;
	self->tiltHeadingDegrees 			= 0;
	self->LSM303D_Temperature 			= 0;
	self->LSM303D_TemperatureDegrees 	= 0;
	self->L3GD20H_Temperature 			= 0;
	self->L3GD20H_TemperatureDegrees	= 0;
	
	// set functions (all these functions need to be defined above, so they're known when it interprets this function)
	self->i2c_write_byte = imu01c_i2c_write_byte;
	self->i2c_read_byte = imu01c_i2c_read_byte;
	self->check_int_range = imu01c_check_int_range;
	self->lsleep = imu01c_lsleep;
	self->twos_comp = imu01c_twos_comp;
	self->update = imu01c_update;
	self->getMag = imu01c_getMag;
	self->getAccel = imu01c_getAccel;
	self->getLSM303DTemperature = imu01c_getLSM303DTemperature;
	self->getL3GD20HTemperature = imu01c_getL3GD20HTemperature;
	self->getHeading = imu01c_getHeading;
	self->getGyro = imu01c_getGyro;
	self->getTiltHeading = imu01c_getTiltHeading;
	self->isMagReady = imu01c_isMagReady;
	
	// lsm303d
	
	// setup communication with magnetometer
	if (self->i2c_lsm303d_addr == 0) {
		if ((self->i2c_lsm303d_addr = open("/dev/i2c-1", O_RDWR)) < 0) {
			fprintf(stderr, "%s : %d : %s failed to open the i2c bus. \n", __FILE__, __LINE__, __func__);
			return false;
		}
		if (ioctl(self->i2c_lsm303d_addr, I2C_SLAVE, LSM303D_ADDRESS) < 0) {
			fprintf(stderr, "%s : %d : %s failed to access the i2c slave at 0x1D. \n", __FILE__, __LINE__, __func__);
			return false;
		}
		// Check functionalities
		unsigned long mFuncs;
		if (ioctl(self->i2c_lsm303d_addr, I2C_FUNCS, &mFuncs) < 0) {
			fprintf(stderr, "%s : %d : %s could not get the adapter functionality matrix (errno %s). \n", __FILE__, __LINE__, __func__, strerror(errno));
			return false;
		}
		if (!(mFuncs & I2C_FUNC_SMBUS_READ_BYTE)) {
			fprintf(stderr, "%s : %d : %s %s SMBus receive byte %s). \n", __FILE__, __LINE__, __func__, MISSING_FUNC_1, MISSING_FUNC_2);
			return false;
		}
		if (!(mFuncs & I2C_FUNC_SMBUS_WRITE_BYTE)) {
			fprintf(stderr, "%s : %d : %s %s SMBus send byte %s). \n", __FILE__, __LINE__, __func__, MISSING_FUNC_1, MISSING_FUNC_2);
			return false;
		}
		if (!(mFuncs & I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
			fprintf(stderr, "%s : %d : %s %s SMBus read byte %s). \n", __FILE__, __LINE__, __func__, MISSING_FUNC_1, MISSING_FUNC_2);
			return false;
		}
		if (!(mFuncs & I2C_FUNC_SMBUS_READ_WORD_DATA)) {
			fprintf(stderr, "%s : %d : %s %s SMBus read word %s). \n", __FILE__, __LINE__, __func__, MISSING_FUNC_1, MISSING_FUNC_2);
			return false;
		}
	}
		
	// initialise the Accelerometer - all axis enabled 50hz
	if (!self->i2c_write_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_CTRL1, 0x57))	// 0x57 = ODR=50hz, all accel axes on ## maybe 0x27 is Low Res?
		fprintf(stderr, "%s : %d : %s : unable to initialise accelerometer. \n", __FILE__, __LINE__, __func__);
	
	// set Accelerometer to continuous updating
	if (!self->i2c_write_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_CTRL2, 0x00))	// set full scale +/- 2g
		fprintf(stderr, "%s : %d : %s : unable to set full scale +/- 2g. \n", __FILE__, __LINE__, __func__);
	
	if (!self->i2c_write_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_CTRL3, 0x00))	// no interrupt
		fprintf(stderr, "%s : %d : %s : unable to set fno interrupt. \n", __FILE__, __LINE__, __func__);
	if (!self->i2c_write_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_CTRL4, 0x00))	// no interrupt
		fprintf(stderr, "%s : %d : %s : unable to set no interrupt. \n", __FILE__, __LINE__, __func__);
		
	if (!self->i2c_write_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_CTRL5, 0xC0))	// 0x10 = mag 50Hz output rate and enabled temperature
		fprintf(stderr, "%s : %d : %s : unable to set mag 50Hz output rate and enabled temperature. \n", __FILE__, __LINE__, __func__);
		
	// enable temperature sensor at 75hz
	if (!self->i2c_write_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_CTRL6, LSM303D_MAG_SCALE_4))
		fprintf(stderr, "%s : %d : %s : unable to set Magnetic Scale +/4 Guass. \n", __FILE__, __LINE__, __func__);
	
	// initialise the Magnetometer
	if (!self->i2c_write_byte(self, self->i2c_lsm303d_addr, LSM303D_REGISTER_CTRL7, 0x00))
		fprintf(stderr, "%s : %d : %s : unable to initialise magnetometer. \n", __FILE__, __LINE__, __func__);
	
	// l3gd20h
	
	// setup gyroscope
	if (self->i2c_l3gd20h_addr == 0) {
		if ((self->i2c_l3gd20h_addr = open("/dev/i2c-1", O_RDWR)) < 0) {
			fprintf(stderr, "%s : %d : %s failed to open the i2c bus. \n", __FILE__, __LINE__, __func__);
			return false;
		}
		if (ioctl(self->i2c_l3gd20h_addr, I2C_SLAVE, L3GD20H_ADDRESS) < 0) {
			fprintf(stderr, "%s : %d : %s failed to access the i2c slave at 0x6B. \n", __FILE__, __LINE__, __func__);
			return false;
		}
		// Check functionalities
		unsigned long mFuncs;
		if (ioctl(self->i2c_l3gd20h_addr, I2C_FUNCS, &mFuncs) < 0) {
			fprintf(stderr, "%s : %d : %s could not get the adapter functionality matrix (errno %s). \n", __FILE__, __LINE__, __func__, strerror(errno));
			return false;
		}
		if (!(mFuncs & I2C_FUNC_SMBUS_READ_BYTE)) {
			fprintf(stderr, "%s : %d : %s %s SMBus receive byte %s). \n", __FILE__, __LINE__, __func__, MISSING_FUNC_1, MISSING_FUNC_2);
			return false;
		}
		if (!(mFuncs & I2C_FUNC_SMBUS_WRITE_BYTE)) {
			fprintf(stderr, "%s : %d : %s %s SMBus send byte %s). \n", __FILE__, __LINE__, __func__, MISSING_FUNC_1, MISSING_FUNC_2);
			return false;
		}
		if (!(mFuncs & I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
			fprintf(stderr, "%s : %d : %s %s SMBus read byte %s). \n", __FILE__, __LINE__, __func__, MISSING_FUNC_1, MISSING_FUNC_2);
			return false;
		}
		if (!(mFuncs & I2C_FUNC_SMBUS_READ_WORD_DATA)) {
			fprintf(stderr, "%s : %d : %s %s SMBus read word %s). \n", __FILE__, __LINE__, __func__, MISSING_FUNC_1, MISSING_FUNC_2);
			return false;
		}
	}
	
	// initialise the gyroscope
	if (!self->i2c_write_byte(self, self->i2c_l3gd20h_addr, L3GD20H_REGISTER_CTRL1, 0x0F))
		fprintf(stderr, "%s : %d : %s : unable to initialise gyroscope. \n", __FILE__, __LINE__, __func__);
	
	self->is_setup = true;
	
	return true;
}