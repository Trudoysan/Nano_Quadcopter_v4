

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "num.h"

//#include "commander.h"
#include "esp_log.h"
#include "pid.h"
#include "num.h"
#include "position_controller.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct pidInit_s {
	float kp;
	float ki;
	float kd;
};

struct pidAxis_s {
	PidObject pid;

	struct pidInit_s init;
	stab_mode_t previousMode;
	float setpoint;

	float output;
};

struct this_s {
	struct pidAxis_s pidVX;
	struct pidAxis_s pidVY;
	struct pidAxis_s pidVZ;

	struct pidAxis_s pidX;
	struct pidAxis_s pidY;
	struct pidAxis_s pidZ;

	uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
	uint16_t thrustMin;  // Minimum thrust value to output
};

// Maximum roll/pitch angle permited
static float rpLimit = 20;
static float rpLimitOverhead = 1.10f;
// Velocity maximums
static float xyVelMax = 1.0f;
static float zVelMax = 1.0f;
static float velMaxOverhead = 1.10f;
//static const float thrustScale = 1000.0f;
static const float thrustScale = 1000.0f;

#define DT (float)(1.0f/POSITION_RATE)
#define POSITION_LPF_CUTOFF_FREQ 20.0f
#define POSITION_LPF_ENABLE true

#ifndef UNIT_TEST
static struct this_s this = { .pidVX = { .init = { .kp = 25.0f, .ki = 1.0f, .kd = 0.0f, }, .pid.dt = DT , },

.pidVY = { .init = { .kp = 25.0f, .ki = 1.0f, .kd = 0.0f, }, .pid.dt = DT , },

.pidVZ = { .init = { .kp = 22, .ki = 15, .kd = 0, }, .pid.dt = DT , },

.pidX = { .init = { .kp = 1.9f, .ki = 0.1f, .kd = 0, }, .pid.dt = DT , },

.pidY = { .init = { .kp = 1.9f, .ki = 0.1f, .kd = 0, }, .pid.dt = DT , },

.pidZ = { .init = { .kp = 1.6f, .ki = 0.5, .kd = 0, }, .pid.dt = DT , },

//thrustBase should just lift the drone
#ifdef CONFIG_MOTOR_BRUSHED_715
  #ifdef CONFIG_TARGET_ESP32_S2_DRONE_V1_2
  .thrustBase = 42000,
  .thrustMin  = 8000,
  #else
  .thrustBase = 36000,
  .thrustMin  = 20000,
  #endif
#else
	//	.thrustBase = 24000, .thrustMin = 5000,
		.thrustBase = 10000, .thrustMin = 5000,
#endif

		};
#endif

void positionControllerInit() {
	pidInit(&this.pidX.pid, this.pidX.setpoint, this.pidX.init.kp, this.pidX.init.ki, this.pidX.init.kd, this.pidX.pid.dt, POSITION_RATE,
	POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
	pidInit(&this.pidY.pid, this.pidY.setpoint, this.pidY.init.kp, this.pidY.init.ki, this.pidY.init.kd, this.pidY.pid.dt, POSITION_RATE,
	POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
	pidInit(&this.pidZ.pid, this.pidZ.setpoint, this.pidZ.init.kp, this.pidZ.init.ki, this.pidZ.init.kd, this.pidZ.pid.dt, POSITION_RATE,
	POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);

	pidInit(&this.pidVX.pid, this.pidVX.setpoint, this.pidVX.init.kp, this.pidVX.init.ki, this.pidVX.init.kd, this.pidVX.pid.dt,
	POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
	pidInit(&this.pidVY.pid, this.pidVY.setpoint, this.pidVY.init.kp, this.pidVY.init.ki, this.pidVY.init.kd, this.pidVY.pid.dt,
	POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
	pidInit(&this.pidVZ.pid, this.pidVZ.setpoint, this.pidVZ.init.kp, this.pidVZ.init.ki, this.pidVZ.init.kd, this.pidVZ.pid.dt,
	POSITION_RATE, POSITION_LPF_CUTOFF_FREQ, POSITION_LPF_ENABLE);
	ESP_LOGI("positionControllerInit", "thrustBase = %d,thrustMin  = %d", this.thrustBase, this.thrustMin);
}

static float runPid(float input, struct pidAxis_s *axis, float setpoint, float dt) {
	axis->setpoint = setpoint;

	pidSetDesired(&axis->pid, axis->setpoint);
	return pidUpdate(&axis->pid, input, true);
}

void positionController(float *thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state) {
	this.pidX.pid.outputLimit = xyVelMax * velMaxOverhead;
	this.pidY.pid.outputLimit = xyVelMax * velMaxOverhead;
	// The ROS landing detector will prematurely trip if
	// this value is below 0.5
	this.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f) * velMaxOverhead;

// otoci smer pro rychlost
	setpoint->velocity.x = runPid(state->position.x, &this.pidX, setpoint->position.x, DT);
	setpoint->velocity.y = runPid(state->position.y, &this.pidY, setpoint->position.y, DT);
	setpoint->velocity.z = runPid(state->position.z, &this.pidZ, setpoint->position.z, DT);

	velocityController(thrust, attitude, setpoint, state);
}

void velocityController(float *thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state) {

	// Roll and Pitch
	float pitchRaw = runPid(state->velocity.x, &this.pidVX, setpoint->velocity.x, DT);
	float rollRaw = runPid(state->velocity.y, &this.pidVY, setpoint->velocity.y, DT);
	//ESP_LOGI(" ", "%f %f %f", state->velocity.x, setpoint->velocity.x, pitchRaw);
	float yawRad = state->attitude.yaw * (float) M_PI / 180;
	setpoint->attitude.roll = (rollRaw * cosf(yawRad)) - (pitchRaw * sinf(yawRad));
	setpoint->attitude.pitch = (pitchRaw * cosf(yawRad)) + (rollRaw * sinf(yawRad));

	//ESP_LOGI(" ", "%f %f %f %f %f", state->position.x, setpoint->velocity.x, state->attitude.yaw, pitchRaw, attitude->pitch);
	setpoint->attitude.roll = constrain(setpoint->attitude.roll, -rpLimit, rpLimit);
	setpoint->attitude.pitch = constrain(setpoint->attitude.pitch, -rpLimit, rpLimit);

	// Thrust
	float thrustRaw = runPid(state->velocity.z, &this.pidVZ, setpoint->velocity.z, DT);
	// Scale the thrust and add feed forward term
	*thrust = thrustRaw * thrustScale + this.thrustBase;
	// Check for minimum thrust
	if (*thrust < this.thrustMin) {
		*thrust = this.thrustMin;
	}
}

void positionControllerResetAllPID() {
	pidReset(&this.pidX.pid);
	pidReset(&this.pidY.pid);
	pidReset(&this.pidZ.pid);
	pidReset(&this.pidVX.pid);
	pidReset(&this.pidVY.pid);
	pidReset(&this.pidVZ.pid);
}
