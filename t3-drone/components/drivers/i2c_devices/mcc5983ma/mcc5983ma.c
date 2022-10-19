/**
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file ms5611.c
 * Driver for the ms5611 pressure sensor from measurement specialties.
 * Datasheet at http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
 *
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mcc5983ma.h"
#include "i2cdev.h"
#include "utils.h"
#include "math.h"
#include "config.h"
#include "static_mem.h"
#include "system.h"
#include "esp_log.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static uint8_t buffer[14];
static bool isInit;
#define MAG_FREQ 200
float magBias[3] = { 0, 0, 0 }, magScale[3] = { 1, 1, 1 }, magOffset[3] = { 0 };

void mcc5983Task(void *arg);
static int16_t magnetMMC_read(void);
static void offsetBias(void);
static void MMC5983MAgetOffset(void);
static void magnetMMC_setup(void);

static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(int16_t));

void mcc5983Init(void) {
	if (isInit)
		return;

	I2Cx = I2C0_DEV;
	devAddr = MMC5983MA_ADDRESS;

	MMC5983MAgetOffset();
	magnetMMC_setup();

	//offsetBias();
	magBias[0] = 0.75;
	magBias[1] = -0.82;
	magBias[2] = 0.35;
	magScale[0] = 0.75;
	magScale[1] = 0.8;
	magScale[2] = 2.0;

	magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);

	xTaskCreate(mcc5983Task, MMC5983_TASK_NAME, MMC5983_TASK_STACKSIZE, NULL, MMC5983_TASK_PRI, NULL);

	isInit = true;
}

void mcc5983Task(void *arg) {

	systemWaitStart();
	int16_t angle;
	TickType_t osTick = xTaskGetTickCount();
	while (1) {
		vTaskDelayUntil(&osTick, F2T(MAG_FREQ));
		angle = magnetMMC_read();
		xQueueOverwrite(magnetometerDataQueue, &angle);
	}
}
bool sensorReadMag(int16_t *angle) {
	return (pdTRUE == xQueuePeek(magnetometerDataQueue, angle, 0));
}
static void MMC5983MAgetOffset(void) {
	//uint8_t rawData[6] = { 0 };  // x/y/z mag register data stored here
	uint16_t data_set[3] = { 0 }, data_reset[3] = { 0 };
	uint8_t temp;

	//powerDown();
	//uint8_t temp = _i2c_bus->readByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2); // read register contents
	i2cdevReadByte(I2Cx, devAddr, MMC5983MA_CONTROL_2, buffer);
	temp = (uint8_t) buffer[0];

	//_i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, temp & ~(0x07)); // clear lowest four bits
	i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_CONTROL_2, temp & ~(0x07));
	vTaskDelay(M2T(20));

	// SET(); // enable set current
	//   _i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x08);
	i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_CONTROL_0, 0x08);
	vTaskDelay(M2T(1));

	//_i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01);  //enable one-time mag measurement
	i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_CONTROL_0, 0x01);
	vTaskDelay(M2T(10));

	//_i2c_bus->readBytes(MMC5983MA_ADDRESS, MMC5983MA_XOUT_0, 6, &rawData[0]);  // Read the 6 raw data registers into data array
	i2cdevReadReg8(I2Cx, devAddr, MMC5983MA_XOUT_0, 6, buffer);
	data_set[0] = buffer[0] << 8 | buffer[1]; // x-axis
	data_set[1] = buffer[2] << 8 | buffer[3]; // y-axis
	data_set[2] = buffer[4] << 8 | buffer[5]; // z-axis

	//RESET(); // enable reset current
	//_i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x10);
	i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_CONTROL_0, 0x10);
	vTaskDelay(M2T(1));

	//_i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01);  //enable one-time mag measurement
	i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_CONTROL_0, 0x01);
	vTaskDelay(M2T(10));

	// _i2c_bus->readBytes(MMC5983MA_ADDRESS, MMC5983MA_XOUT_0, 6, &rawData[0]);  // Read the 6 raw data registers into data array
	i2cdevReadReg8(I2Cx, devAddr, MMC5983MA_XOUT_0, 6, buffer);
	data_reset[0] = buffer[0] << 8 | buffer[1]; // x-axis
	data_reset[1] = buffer[2] << 8 | buffer[3]; // y-axis
	data_reset[2] = buffer[4] << 8 | buffer[5]; // z-axis

	for (uint8_t ii = 0; ii < 3; ii++) {
		magOffset[ii] = ((float) data_set[ii] + (float) data_reset[ii]) / 2.0f;
	}
	/*	Serial.print("magOffset:");
	 Serial.print(magOffset[0]);
	 Serial.print(",");
	 Serial.print(magOffset[1]);
	 Serial.print(",");
	 Serial.print(magOffset[2]);
	 Serial.println();*/
}

static void magnetMMC_setup(void) {

	//RESET(); // enable reset current
	//_i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x10);
	i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_CONTROL_0, 0x10);
	vTaskDelay(M2T(1));

	// SET(); // enable set current
	//   _i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x08);
	i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_CONTROL_0, 0x08);
	vTaskDelay(M2T(1));

	// enable data ready interrupt (bit2 == 1), enable auto set/reset (bit 5 == 1)
	// this set/reset is a low current sensor offset measurement for normal use
	//_i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x20 | 0x04);
	i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_CONTROL_0, 0x20);

	// set magnetometer bandwidth
	//_i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, MBW);
	i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_CONTROL_1, MBW_200Hz);

	// enable continuous measurement mode (bit 3 == 1), set sample rate
	// enable automatic Set/Reset (bit 7 == 1), set set/reset rate
	// this set/reset is a high-current "deGaussing" that should be used only to recover from
	// high magnetic field detuning of the magnetoresistive film
	// _i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | 0x08 | MODR);
	//_i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x08 | MODR);
	i2cdevWriteByte(I2Cx, devAddr, MMC5983MA_CONTROL_2, MODR_200Hz | 0x08);
}

static void MMC5983MAreadData(uint32_t *destination) {
	//uint8_t rawData[7];  // x/y/z mag register data stored here
	//_i2c_bus->readBytes(MMC5983MA_ADDRESS, MMC5983MA_XOUT_0, 7, &rawData[0]);  // Read the 7 raw data registers into data array
	i2cdevReadReg8(I2Cx, devAddr, MMC5983MA_XOUT_0, 7, buffer);
	destination[0] = (uint32_t) (buffer[0] << 10 | buffer[1] << 2 | (buffer[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value
	destination[1] = (uint32_t) (buffer[2] << 10 | buffer[3] << 2 | (buffer[6] & 0x30) >> 4); // Turn the 18 bits into a unsigned 32-bit value
	destination[2] = (uint32_t) (buffer[4] << 10 | buffer[5] << 2 | (buffer[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value
}

static void offsetBias(void) {
	int32_t bias[3] = { 0, 0, 0 }, scale[3] = { 0, 0, 0 };
	int32_t mag_max[3] = { -262143, -262143, -262143 }, mag_min[3] = { 262143, 262143, 262143 };
	uint32_t mag_temp[3] = { 0, 0, 0 }, magOffset = 131072;
	float _mRes = 1.0f / 16384.0f;      // mag sensitivity if using 18 bit data

//  Serial.println("Calculate mag offset bias: move all around to sample the complete response surface!");
//  delay(4000);
	ESP_LOGI("offsetBias", "Calculate mag offset bias: move all around to sample the complete response surface!");
	vTaskDelay(M2T(2500));

	//  for (int ii = 0; ii < 4000; ii++)
	for (int ii = 0; ii < 400; ii++) {
		MMC5983MAreadData(mag_temp);
		for (int jj = 0; jj < 3; jj++) {
			if ((int32_t) (mag_temp[jj] - magOffset) > mag_max[jj])
				mag_max[jj] = (int32_t) (mag_temp[jj] - magOffset);
			if ((int32_t) (mag_temp[jj] - magOffset) < mag_min[jj])
				mag_min[jj] = (int32_t) (mag_temp[jj] - magOffset);
		}
		vTaskDelay(M2T(12));
	}

	// Get hard iron correction
	bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
	bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
	bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

	magBias[0] = (float) (bias[0]) * _mRes;  // save mag biases in G for main program
	magBias[1] = (float) (bias[1]) * _mRes;
	magBias[2] = (float) (bias[2]) * _mRes;

	// Get soft iron correction estimate
	scale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
	scale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
	scale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

	float avg_rad = scale[0] + scale[1] + scale[2];
	avg_rad /= 3.0f;

	magScale[0] = avg_rad / ((float) scale[0]);
	magScale[1] = avg_rad / ((float) scale[1]);
	magScale[2] = avg_rad / ((float) scale[2]);
	ESP_LOGI(" ", "%d %d %d %d %d %d %f %f %f %f %f %f ", mag_min[0], mag_min[1], mag_min[2], mag_max[0], mag_max[1], mag_max[2],
			magBias[0], magBias[1], magBias[2], magScale[0], magScale[1], magScale[2]);
	/*
	 Serial.println("Mag Calibration done!");
	 Serial.print("mag_min:");
	 Serial.print(mag_min[0]);
	 Serial.print(",");
	 Serial.print(mag_min[1]);
	 Serial.print(",");
	 Serial.print(mag_min[2]);
	 Serial.print("mag_max:");
	 Serial.print(mag_max[0]);
	 Serial.print(",");
	 Serial.print(mag_max[1]);
	 Serial.print(",");
	 Serial.print(mag_max[2]);
	 Serial.print("magBias:");
	 Serial.print(magBias[0]);
	 Serial.print(",");
	 Serial.print(magBias[1]);
	 Serial.print(",");
	 Serial.print(magBias[2]);
	 Serial.print("  magScale:");
	 Serial.print(magScale[0]);
	 Serial.print(",");
	 Serial.print(magScale[1]);
	 Serial.print(",");
	 Serial.print(magScale[2]);
	 Serial.println();*/
}

static int16_t magnetMMC_read(void) {

	float MMC5983MA_offset = 131072.0f;
	uint32_t mag_temp[3] = { 0, 0, 0 };
	float mRes = 1.0f / 16384.0f;
	float mx, my, mz;

	MMC5983MAreadData(mag_temp);

	// Now we'll convert mag data into actual G's
	mx = ((float) mag_temp[0] - MMC5983MA_offset) * mRes - magBias[0]; // get actual G value
	my = ((float) mag_temp[1] - MMC5983MA_offset) * mRes - magBias[1];
	mz = ((float) mag_temp[2] - MMC5983MA_offset) * mRes - magBias[2];
	mx *= magScale[0];
	my *= magScale[1];
	mz *= magScale[2];

	int16_t angleMag = atan2(mx, my) * 180.0 / M_PI;
	return angleMag < 0 ? 360 + angleMag : angleMag;
	//return mag_temp[0];
}
