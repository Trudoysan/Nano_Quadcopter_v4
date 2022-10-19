

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//#include "log.h"
//#include "param.h"
#include "num.h"
#include "position_estimator.h"
#include "utils.h"

#define G 9.81f;

struct selfstateEA_s {
	float estimatedZ; // The current Z estimate, has same offset as asl
	float velocityZ; // Vertical speed (world frame) integrated from vertical acceleration (m/s)
	float estAlphaZrange;
	float estAlphaAsl;
	float velocityFactor;
	float vAccDeadband;
	;
	// Vertical acceleration deadband
	float velZAlpha;   // Blending factor to avoid vertical speed to accumulate error
	float estimatedVZ;
};

//static struct selfstateEA_s stateEA = { .estimatedZ = 0.0f, .velocityZ = 0.0f, .estAlphaZrange = 0.90f, .estAlphaAsl = 0.997f,
//		.velocityFactor = 1.0f, .vAccDeadband = 0.04f, .velZAlpha = 0.995f, .estimatedVZ = 0.0f, };
static struct selfstateEA_s stateEA = { .estimatedZ = 0.0f, .velocityZ = 0.0f, .estAlphaZrange = 0.80f, .estAlphaAsl = 0.997f,
		.velocityFactor = 1.0f, .vAccDeadband = 0.04f, .velZAlpha = 0.995f, .estimatedVZ = 0.0f, };

static void positionEstimateInternal(state_t *estimate, const sensorData_t *sensorData, /*const tofMeasurement_t *tofMeasurement,*/float dt,
		uint32_t tick);
static void positionUpdateVelocityInternal(float accWZ, float dt);

void positionEstimate(state_t *estimate, const sensorData_t *sensorData,/* const tofMeasurement_t *tofMeasurement, */float dt,
		uint32_t tick) {
	positionEstimateInternal(estimate, sensorData, /*tofMeasurement,*/dt, tick);
}

void positionUpdateVelocity(float accWZ, float dt) {
	positionUpdateVelocityInternal(accWZ, dt);
}

static void positionEstimateInternal(state_t *estimate, const sensorData_t *sensorData/*, const tofMeasurement_t *tofMeasurement*/
, float dt, uint32_t tick) {
	float filteredZ;
	static float prev_estimatedZ = 0;

	/*	const uint32_t MAX_SAMPLE_AGE = M2T(50);

	 uint32_t now = xTaskGetTickCount();
	 bool isSampleUseful = ((now - sensorData->interruptTimestamp) <= MAX_SAMPLE_AGE);

	 if (isSampleUseful) {
	 */		// IIR filter zrange
	if (stateEA.estimatedZ == 0.0f) {
		filteredZ = sensorData->asl;
	} else {
		filteredZ = (stateEA.estAlphaZrange) * stateEA.estimatedZ + (1.0f - stateEA.estAlphaZrange) * sensorData->asl;
		// Use zrange as base and add velocity changes.
	}
	stateEA.estimatedZ = filteredZ + (stateEA.velocityFactor * stateEA.velocityZ * dt);

	/*	} else {
	 // FIXME: A bit of an hack to init IIR filter
	 if (stateEA.estimatedZ == 0.0f) {
	 filteredZ = sensorData->asl;
	 } else {
	 // IIR filter asl
	 filteredZ = (stateEA.estAlphaAsl) * stateEA.estimatedZ + (1.0f - stateEA.estAlphaAsl) * sensorData->asl;
	 }
	 // Use asl as base and add velocity changes.
	 stateEA.estimatedZ = filteredZ + (stateEA.velocityFactor * stateEA.velocityZ * dt);
	 }*/

//	estimate->position.x = 0.0f;
//	estimate->position.y = 0.0f;
	estimate->position.z = stateEA.estimatedZ;
	estimate->velocity.z = (stateEA.estimatedZ - prev_estimatedZ) / dt;
//	stateEA.estimatedVZ = estimate->velocity.z;
	prev_estimatedZ = stateEA.estimatedZ;
}

static void positionUpdateVelocityInternal(float accWZ, float dt) {
	stateEA.velocityZ += deadband(accWZ, stateEA.vAccDeadband) * dt * G;
	stateEA.velocityZ *= stateEA.velZAlpha;
}

