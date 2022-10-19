

#include "kalman_core_v2.h"
#include "sensor_spl06.h"
#include "physicalConstants.h"
#include "math3d.h"
#include "xtensa_math.h"
#include "esp_log.h"

static kalmanCoreData_t2 coreData;

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

static float initialX = 0.0;
static float initialY = 0.0;

static const float stdDevInitialPosition_xy = 100;
static const float stdDevInitialVelocity = 0.01;

static float procNoiseAcc_xy = 0.5f;
static float procNoiseVel = 0;
static float procNoisePos = 0;

void kalmanCoreInit2(state_t *state) {

	coreData.S2[KC_STATE_X] = initialX;
	coreData.S2[KC_STATE_Y] = initialY;
	coreData.S2[KC_STATE_PX] = initialX;
	coreData.S2[KC_STATE_PY] = initialY;

	// initialize state variances
	coreData.P2[KC_STATE_X] = powf(stdDevInitialPosition_xy, 2);
	coreData.P2[KC_STATE_Y] = powf(stdDevInitialPosition_xy, 2);

	coreData.P2[KC_STATE_PX] = powf(stdDevInitialVelocity, 2);
	coreData.P2[KC_STATE_PY] = powf(stdDevInitialVelocity, 2);

	state->position.x = 0;
	state->position.y = 0;
	state->velocity.x = 0;
	state->velocity.y = 0;
	ESP_LOGI("kalmanCoreInit2", "init");
}

//
// state - data prijata (acc.x, acc.y - uz rotovana viz calculateRotation funkce) a data publikovana (position.x a.y a velocity.x a.y)
//dx a dx - difference v m za cas dt
// stDev - chyba mereni, oni meli 0.25 a davali to na druhou, musime odzkouset

void kalman2(state_t *state, float dx, float dy, float stDev, float dt) {
	static float K2[KC_STATE_DIM];
	float dt2 = dt * dt;

	//Calculating new predict
	coreData.S2[KC_STATE_X] += (dt * coreData.S2[KC_STATE_PX]);// + (0.5f * dt2 * state->acc.x);
	coreData.S2[KC_STATE_Y] += (dt * coreData.S2[KC_STATE_PY]);// + (0.5f * dt2 * state->acc.y);
	//coreData.S2[KC_STATE_PX] += (dt * state->acc.x);
	//coreData.S2[KC_STATE_PY] += (dt * state->acc.y);

	//Calculating new coveriance
	coreData.P2[KC_STATE_X] += (dt2 * coreData.P2[KC_STATE_PX]);
	coreData.P2[KC_STATE_Y] += (dt2 * coreData.P2[KC_STATE_PY]);

	// add noise
	coreData.P2[KC_STATE_X] += powf(procNoiseAcc_xy * dt2 + procNoiseVel * dt + procNoisePos, 2); // add process noise on position
	coreData.P2[KC_STATE_Y] += powf(procNoiseAcc_xy * dt2 + procNoiseVel * dt + procNoisePos, 2); // add process noise on position
	coreData.P2[KC_STATE_PX] += powf(procNoiseAcc_xy * dt + procNoiseVel, 2); // add process noise on velocity
	coreData.P2[KC_STATE_PY] += powf(procNoiseAcc_xy * dt + procNoiseVel, 2); // add process noise on velocity

	//Kalman Gain
	K2[KC_STATE_X] = coreData.P2[KC_STATE_X] / (coreData.P2[KC_STATE_X] + stDev);
	K2[KC_STATE_Y] = coreData.P2[KC_STATE_Y] / (coreData.P2[KC_STATE_Y] + stDev);
	K2[KC_STATE_PX] = coreData.P2[KC_STATE_PX] / (coreData.P2[KC_STATE_PX] + stDev / dt); // chyba mereni jako podil chyby mereni vzdalenosti za cas, netusim jestli OK
	K2[KC_STATE_PY] = coreData.P2[KC_STATE_PY] / (coreData.P2[KC_STATE_PY] + stDev / dt);

	//Final calculate
	// (state->position.x - dx) je nova zmerena poloha
	// coreData.S2[KC_STATE_X] je predikce
	state->position.x += K2[KC_STATE_X] * ((state->position.x + dx) - coreData.S2[KC_STATE_X]);
	state->position.y += K2[KC_STATE_Y] * ((state->position.y + dy) - coreData.S2[KC_STATE_Y]);
	state->velocity.x += K2[KC_STATE_PX] * ((dx / dt) - coreData.S2[KC_STATE_PX]);
	state->velocity.y += K2[KC_STATE_PY] * ((dy / dt) - coreData.S2[KC_STATE_PY]);

	//ESP_LOGI(" ", "%f %f %f %f %f %f %f", dx, coreData.S2[KC_STATE_X], coreData.P2[KC_STATE_X], K2[KC_STATE_X], dt, state->position.x,dt * coreData.S2[KC_STATE_PX]);

	coreData.S2[KC_STATE_X] = state->position.x;
	coreData.S2[KC_STATE_Y] = state->position.y;
	coreData.S2[KC_STATE_PX] = state->velocity.x;
	coreData.S2[KC_STATE_PY] = state->velocity.y;

	//Updating coveriance matrix
	coreData.P2[KC_STATE_X] *= (1 - K2[KC_STATE_X]);
	coreData.P2[KC_STATE_Y] *= (1 - K2[KC_STATE_Y]);
	coreData.P2[KC_STATE_PX] *= (1 - K2[KC_STATE_PX]);
	coreData.P2[KC_STATE_PY] *= (1 - K2[KC_STATE_PY]);

}

