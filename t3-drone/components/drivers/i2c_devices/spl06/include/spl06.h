

#ifndef COMPONENTS_DRIVERS_I2C_DEVICES_SPL06_INCLUDE_SPL06_H_
#define COMPONENTS_DRIVERS_I2C_DEVICES_SPL06_INCLUDE_SPL06_H_

#include "i2cdev.h"

#define SPL06_ADDRESS    0x76

bool spl06Init(I2C_Dev *i2cPort);
float spl06Read(void);

#endif /* COMPONENTS_DRIVERS_I2C_DEVICES_SPL06_INCLUDE_SPL06_H_ */
