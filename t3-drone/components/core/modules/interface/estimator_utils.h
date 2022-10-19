

#ifndef COMPONENTS_CORE_MODULES_INTERFACE_ESTIMATOR_UTILS_H_
#define COMPONENTS_CORE_MODULES_INTERFACE_ESTIMATOR_UTILS_H_


void calculateRotationMatrix(state_t *state);
void calculateRotation(float *accX,float *accY,float *accZ, float X, float Y, float Z, state_t *state);


#endif /* COMPONENTS_CORE_MODULES_INTERFACE_ESTIMATOR_UTILS_H_ */
