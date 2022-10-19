

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "math.h"
#include "i2cdev.h"
#include "spl06.h"
#include "utils.h"

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static uint8_t buffer[18];
static bool isInit;

static void baro_setup(void);
static bool spl06ID(void);

bool spl06Init(I2C_Dev *i2cPort) {
	if (isInit) {
		return true;
	}

	I2Cx = i2cPort;
	devAddr = SPL06_ADDRESS;
	if(spl06ID() == false)
		return false;
	baro_setup();
	isInit = true;
	return true;

}

static float kt;
static float kp;
static int32_t press_raw;
static int32_t temp_raw;
static struct {
	int16_t c0, c1;
	int32_t c00, c10;
	int16_t c01, c11, c20, c21, c30;
} coe;

static bool spl06ID(void) {
	i2cdevReadByte(I2Cx, devAddr, 0x0d, buffer);
	if (buffer[0] != 0x10) {
		ESP_LOGW("baro_setup", "Wrong sensor");
		return false;
	}
	return true;
}

static void baro_setup(void) {

	i2cdevReadReg8(I2Cx, devAddr, 0x10, 18, buffer);

	coe.c0 = (uint16_t) buffer[0] << 4 | (uint16_t) buffer[1] >> 4;
	coe.c0 = (coe.c0 & 1 << 11) ? (0xf000 | coe.c0) : coe.c0;

	coe.c1 = (uint16_t) (buffer[1] & 0x0f) << 8 | (uint16_t) buffer[2];
	coe.c1 = (coe.c1 & 1 << 11) ? (0xf000 | coe.c1) : coe.c1;

	coe.c00 = (uint32_t) buffer[3] << 12 | (uint32_t) buffer[4] << 4
			| (uint16_t) buffer[5] >> 4;
	coe.c00 = (coe.c00 & 1 << 19) ? (0xfff00000 | coe.c00) : coe.c00;

	coe.c10 = (uint32_t) (buffer[5] & 0x0f) << 16 | (uint32_t) buffer[6] << 8
			| (uint32_t) buffer[7];
	coe.c10 = (coe.c10 & 1 << 19) ? (0xfff00000 | coe.c10) : coe.c10;

	coe.c01 = (uint16_t) buffer[8] << 8 | buffer[9];
	coe.c11 = (uint16_t) buffer[10] << 8 | buffer[11];
	coe.c20 = (uint16_t) buffer[12] << 8 | buffer[13];
	coe.c21 = (uint16_t) buffer[14] << 8 | buffer[15];
	coe.c30 = (uint16_t) buffer[16] << 8 | buffer[17];

	i2cdevWriteByte(I2Cx, devAddr, 0x06, (7 << 4 | 6));
	kp = 1040384;
//	i2cdevWriteByte(I2Cx, devAddr, 0x06, 0x26);
//	kp = 1040384;

	i2cdevWriteByte(I2Cx, devAddr, 0x07, (1 << 7 | 3 << 4 | 3));
	kt = 7864320;
//	i2cdevWriteByte(I2Cx, devAddr, 0x07, 0xA0);
//	kt = 3670016;

	i2cdevWriteByte(I2Cx, devAddr, 0x09, (1 << 2));

	i2cdevWriteByte(I2Cx, devAddr, 0x08, 7);
}

float spl06Read(void) {
	i2cdevReadReg8(I2Cx, devAddr, 0x03, 3, buffer);
	temp_raw = ((uint32_t) buffer[0]) << 16 | buffer[1] << 8 | buffer[2];
	temp_raw = (temp_raw & 1 << 23) ? (0xff000000 | temp_raw) : temp_raw;

	i2cdevReadReg8(I2Cx, devAddr, 0x00, 3, buffer);
	press_raw = ((uint32_t) buffer[0]) << 16 | buffer[1] << 8 | buffer[2];
	press_raw = (press_raw & 1 << 23) ? (0xff000000 | press_raw) : press_raw;

// calculate
	float ftsc = (float) temp_raw / kt;

	float temperature = (float) coe.c0 * 0.5f + (float) coe.c1 * ftsc;
	//  Serial.print( " temperature_raw ");
	//  Serial.print(temperature);

	float fpsc = (float) press_raw / kp;
//  Serial.print( " fpsc ");
//  Serial.print(fpsc);

	float qua2 = (float) coe.c10
			+ fpsc * ((float) coe.c20 + fpsc * (float) coe.c30);
	float qua3 = ftsc * fpsc * ((float) coe.c11 + fpsc * (float) coe.c21);

	float fp = (float) coe.c00 + fpsc * qua2 + ftsc * (float) coe.c01 + qua3;
//  Serial.print( " fp ");
//  Serial.print(fp);
	// altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1

	// tropospheric properties (0-11km) for standard atmosphere
	const float T1 = 15.0f + 273.15f;    // temperature at base height in Kelvin
	const float a = -6.5f / 1000.0f; // temperature gradient in degrees per metre
	const float g = 9.80665f;              // gravity constant in m/s/s
	const float R = 287.05f;               // ideal gas constant in J/kg/K
	const float msl_pressure = 101325.0f;   // in Pa
	float pK = fp / msl_pressure;

	// temperature = (float)_spl.coe.c0 * 0.5f + (float)_spl.coe.c1 * ftsc;

	return ((((powf(pK, (-(a * R) / g))) * T1) - T1) / a);
	//return ((float) press_raw);
}

