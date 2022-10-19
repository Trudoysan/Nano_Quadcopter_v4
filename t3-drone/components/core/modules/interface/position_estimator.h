
#ifndef POSITION_ESTIMATOR_H_
#define POSITION_ESTIMATOR_H_

#include "stabilizer_types.h"

void positionEstimate(state_t* estimate, const sensorData_t* sensorData/*, const tofMeasurement_t* tofMeasurement*/, float dt, uint32_t tick);
void positionUpdateVelocity(float accWZ, float dt);

#endif /* POSITION_ESTIMATOR_H_ */
