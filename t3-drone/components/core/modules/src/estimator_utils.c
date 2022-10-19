

#include "stabilizer_types.h"
#include "estimator_utils.h"

void calculateRotationMatrix(state_t *state)
{


	state->rotation[0][0] = state->attitudeQuaternion.w * state->attitudeQuaternion.w + state->attitudeQuaternion.x * state->attitudeQuaternion.x - state->attitudeQuaternion.y * state->attitudeQuaternion.y - state->attitudeQuaternion.z * state->attitudeQuaternion.z;
	state->rotation[0][1] = 2 * state->attitudeQuaternion.x * state->attitudeQuaternion.y - 2 * state->attitudeQuaternion.w * state->attitudeQuaternion.z;
	state->rotation[0][2] = 2 * state->attitudeQuaternion.x * state->attitudeQuaternion.z + 2 * state->attitudeQuaternion.w * state->attitudeQuaternion.y;

	state->rotation[1][0] = 2 * state->attitudeQuaternion.x * state->attitudeQuaternion.y + 2 * state->attitudeQuaternion.w * state->attitudeQuaternion.z;
	state->rotation[1][1] = state->attitudeQuaternion.w * state->attitudeQuaternion.w - state->attitudeQuaternion.x * state->attitudeQuaternion.x + state->attitudeQuaternion.y * state->attitudeQuaternion.y - state->attitudeQuaternion.z * state->attitudeQuaternion.z;
	state->rotation[1][2] = 2 * state->attitudeQuaternion.y * state->attitudeQuaternion.z - 2 * state->attitudeQuaternion.w * state->attitudeQuaternion.x;

	state->rotation[2][0] = 2 * state->attitudeQuaternion.x * state->attitudeQuaternion.z - 2 * state->attitudeQuaternion.w * state->attitudeQuaternion.y;
	state->rotation[2][1] = 2 * state->attitudeQuaternion.y * state->attitudeQuaternion.z + 2 * state->attitudeQuaternion.w * state->attitudeQuaternion.x;
	state->rotation[2][2] = state->attitudeQuaternion.w * state->attitudeQuaternion.w - state->attitudeQuaternion.x * state->attitudeQuaternion.x - state->attitudeQuaternion.y * state->attitudeQuaternion.y + state->attitudeQuaternion.z * state->attitudeQuaternion.z;
}


void calculateRotation(float *accX,float *accY,float *accZ, float X, float Y, float Z, state_t *state){

	*accX = state->rotation[0][0] * X + state->rotation[0][1] * Y + state->rotation[0][2] * Z;
    *accY = state->rotation[1][0] * X + state->rotation[1][1] * Y + state->rotation[1][2] * Z;
    *accZ = state->rotation[2][0] * X + state->rotation[2][1] * Y + state->rotation[2][2] * Z;

}
