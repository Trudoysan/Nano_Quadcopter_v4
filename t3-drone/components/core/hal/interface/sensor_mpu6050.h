

#ifndef __SENSOR_MPU6050_H__
#define __SENSOR_MPU6050_H__

#include "imu_types.h"
#include "stabilizer_types.h"


void sensorMpu6050Init(void);
bool sensorsMpu6050Calibrated(void);

void sensorsMpu6050Acquire(sensorData_t *sensors, const uint32_t tick);
void sensorsMpu6050WaitDataReady(void);
bool sensorsMpu6050ReadGyro(Axis3f *gyro);
bool sensorsMpu6050ReadAcc(Axis3f *acc);

#endif // __SENSOR_MPU6050_H__
