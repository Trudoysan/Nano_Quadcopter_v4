//#define DEBUG_MODULE "CTRL_P"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "stabilizer.h"
#include "stabilizer_types.h"
#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_pid.h"
#include "espnow.h"
#include "math3d.h"
#include "esp_log.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

//static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

void controllerPidInit(void) {
	attitudeControllerInit(ATTITUDE_UPDATE_DT);
	positionControllerInit();
}

void controllerPidReset(void) {
	attitudeControllerResetAllPID();
	positionControllerResetAllPID();
}

bool controllerPidTest(void) {
	bool pass = true;

	pass &= attitudeControllerTest();
	printf("controller_Pid_Test = %d", pass);

	return pass;
}

static float capAngle(float angle) {
	float result = angle;

	while (result > 180.0f) {
		result -= 360.0f;
	}

	while (result < -180.0f) {
		result += 360.0f;
	}

	return result;
}

void controllerPid(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		attitudeDesired.yaw = setpoint->attitude.yaw + state->startMag - sensors->mag;
		attitudeDesired.yaw = capAngle(attitudeDesired.yaw);

		attitudeDesired.roll = setpoint->attitude.roll;
		attitudeDesired.pitch = setpoint->attitude.pitch;
	}

	if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
		//upraveno, nastavi setpoint->attitude.roll a pitch podle dx a dy
		positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
	}

	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
// prictu pitch a roll z dx a dy k hodnotam z complem filtru
		attitudeDesired.roll += setpoint->attitude.roll;
		attitudeDesired.pitch += setpoint->attitude.pitch;

		attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw, attitudeDesired.roll,
				attitudeDesired.pitch, attitudeDesired.yaw, &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

		// TODO: Investigate possibility to subtract gyro drift.
		attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z, rateDesired.roll, rateDesired.pitch,
				rateDesired.yaw);

		attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);
		//ESP_LOGI(" ", "%f %f %f %d", state->attitude.roll,attitudeDesired.roll,rateDesired.roll,control->roll);
		control->yaw = -control->yaw;

	}

	control->thrust = actuatorThrust;

	/*	if (control->thrust == 0) {
	 control->thrust = 0;
	 control->roll = 0;
	 control->pitch = 0;
	 control->yaw = 0;

	 attitudeControllerResetAllPID();
	 positionControllerResetAllPID();

	 // Reset the calculated YAW angle for rate control
	 attitudeDesired.yaw = state->attitude.yaw;
	 }*/
}
