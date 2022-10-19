

#include <string.h>
#include "power_distribution.h"
#include "num.h"
#include "motors.h"
#include "espnow.h"

static bool motorSetEnable = false;

static struct {
	uint32_t m1;
	uint32_t m2;
	uint32_t m3;
	uint32_t m4;
} motorPower;

static struct {
	uint16_t m1;
	uint16_t m2;
	uint16_t m3;
	uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void) {
	motorsInit(/*platformConfigGetMotorMapping()*/);
}

bool powerDistributionTest(void) {
	bool pass = true;

	pass &= motorsTest();

	return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop() {
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);
}
static returnData_message returnData;
static int irun = 0;
static uint16_t motorsConv16ToBits(uint16_t bits) {
	return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}
void powerDistribution(const control_t *control) {
#ifdef QUAD_FORMATION_X
	if (control->thrust != 0) {
		int16_t r = -control->roll / 2.0f; // - pridano vsude
		int16_t p = -control->pitch / 2.0f;
		motorPower.m1 = limitThrust(control->thrust - r + p - control->yaw);// + control->yaw);
		motorPower.m2 = limitThrust(control->thrust - r - p + control->yaw);//- control->yaw);
		motorPower.m3 = limitThrust(control->thrust + r - p - control->yaw);//+ control->yaw);
		motorPower.m4 = limitThrust(control->thrust + r + p + control->yaw);//- control->yaw);
	} else {
		motorPower.m1 = 0;
		motorPower.m2 = 0;
		motorPower.m3 = 0;
		motorPower.m4 = 0;
	}
#else // QUAD_FORMATION_NORMAL
/*    motorPower.m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
    motorPower.m3 =  limitThrust(control->thrust - control->pitch +
                               control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + control->roll -
                               control->yaw);*/
  #endif
/*	irun++;
	if (irun > 500) { //  **************************************************************
		returnData.roll = control->roll;
		returnData.pitch = control->pitch;
		returnData.yaw = control->yaw;
		returnData.throttle = control->thrust;
		returnData.motor1 = (uint32_t) motorsConv16ToBits(motorPower.m4);
		returnData.motor2 = (uint32_t) motorsConv16ToBits(motorPower.m1);
		returnData.motor3 = (uint32_t) motorsConv16ToBits(motorPower.m2);
		returnData.motor4 = (uint32_t) motorsConv16ToBits(motorPower.m3);
		xQueueSend(espnow_queueTx, &returnData, 0);
	}

	if (irun > 500) {
		irun = 0;
	}
*//*
	if (motorSetEnable) {
		motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
		motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
		motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
		motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
	} else {
		motorsSetRatio(MOTOR_M1, motorPower.m2);
		motorsSetRatio(MOTOR_M2, motorPower.m3);
		motorsSetRatio(MOTOR_M3, motorPower.m4);
		motorsSetRatio(MOTOR_M4, motorPower.m1);
	}
*/
}

