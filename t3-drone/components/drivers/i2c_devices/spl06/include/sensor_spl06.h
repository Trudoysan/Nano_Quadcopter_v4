

#ifndef COMPONENTS_DRIVERS_I2C_DEVICES_SPL06_INCLUDE_SENSOR_SPL06_H_
#define COMPONENTS_DRIVERS_I2C_DEVICES_SPL06_INCLUDE_SENSOR_SPL06_H_

#include "stdbool.h"

float baro_stddev;
bool useBaroUpdate;

void sensor_spl06Init(void);
bool sensorReadBaro(float *baro);


#endif /* COMPONENTS_DRIVERS_I2C_DEVICES_SPL06_INCLUDE_SENSOR_SPL06_H_ */
