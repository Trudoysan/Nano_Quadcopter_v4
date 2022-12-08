#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "config.h"
#include "static_mem.h"
#include "esp_log.h"
#include "sensor_mpu6050.h"
#include "system.h"
#include "utils.h"
#include "sensor_spl06.h"
#include "estimator_complementary.h"
#include "flowdeck_v1v2.h"
#include "mcc5983ma.h"
#include "kalman_core_v2.h"
#include "commander.h"
#include "controller_pid.h"
#include "power_distribution.h"
#include "mcc5983ma.h"
#include <math.h>

/* #include "freertos/FreeRTOS.h"
 #include "freertos/queue.h"
 #include "sensors.h"
 #include "espnow.h"
 #include "led.h"
 */

static bool isInit;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;
static returnData_message returnData;

bool inTheAir = false;
STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask(void *param);

void stabilizerInit(void) {
	if (isInit)
		return;

	ESP_LOGI("stabilizerInit", "Started");

	//sensorsMpu6050Init(); je v systemu
	//sensor_spl06Init();  je v systemu

	estimatorComplementaryInit();
	kalmanCoreInit2(&state);
	controllerPidInit();
	powerDistributionInit();

	STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

//	emergStop = false;
	isInit = true;
}


static void stabilizerTask(void *param) {
	uint32_t tick;
	uint32_t lastWakeTime;

	//Wait for the system to be fully started to start stabilization loop
	systemWaitStart();

	ESP_LOGI("stabilizerTask", "Wait for sensor calibration...");

	
	lastWakeTime = xTaskGetTickCount();
	while (!sensorsMpu6050Calibrated()) {
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
	}
	sensorReadBaro(&sensorData.asl); // for init
	state.baroReferenceHeight = sensorData.asl;
	sensorReadMag(&sensorData.mag);
	state.startMag = sensorData.mag;
	// Initialize tick to something else then 0
	tick = 1;
	lastKalman = tick;

	ESP_LOGI("stabilizerTask", "Ready to fly");

	while (1) {
		// The sensor should unlock at 1kHz
		sensorsMpu6050WaitDataReady();

		/*	if (tick == 5000) { // jen pro testovani bez controleru
		 ESP_LOGI("stabilizerTask", "In the air");
		 //control.thrust = 15000.0;
		 //inTheAir = true;
		 state.baroReferenceHeight = sensorData.asl;
		 kalmanCoreInit2(&state);
		 }*/

		estimatorComplementary(&state, &sensorData, &control, tick);

		commanderGetSetpoint(&setpoint, &state);

		if (setpoint.position.z > 0.01 && setpoint.flyDron) {
			if (!inTheAir) {
				state.baroReferenceHeight = sensorData.asl;
				state.startMag = sensorData.mag;
				kalmanCoreInit2(&state);
				controllerPidReset();
				inTheAir = true;
			}
			controllerPid(&control, &setpoint, &sensorData, &state, tick);
			if (!(tick % 500)) { //  ***************************************************************
				returnData.a = state.startMag;
				returnData.b = sensorData.mag;
				returnData.c = control.roll;
				returnData.d = control.pitch;
				returnData.e = control.yaw;
				returnData.f = control.thrust;
				returnData.fa = state.position.x;
				returnData.fb = state.position.y;
				returnData.fc = state.position.z;
				returnData.fd = state.attitude.roll;
				returnData.fe = state.attitude.pitch,
				returnData.ff = state.attitude.yaw;
				xQueueSend(espnow_queueTx, &returnData, 0);

				// ESP_LOGI(" ", "%f %d %d", setpoint.attitude.yaw, state.startMag,sensorData.mag);
				// ESP_LOGI(" ", "asl %f, acc %f %f %f, gyro %f %f %f ", asl, sensorData.acc.x, sensorData.acc.y, sensorData.acc.z,
				// sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z);
//			ESP_LOGI(" ", "asl %f, att %f %f %f, pos %f %f %f, vel %f %f %f", sensorData.asl, state.attitude.pitch, state.attitude.roll,
//					state.attitude.yaw, state.position.x, state.position.y, state.position.z, state.velocity.x, state.velocity.y,
//					state.velocity.z);
//			ESP_LOGI(" ", "asl %f, att %f %f %f, sensor %f %f %f", sensorData.asl, state.attitude.pitch, state.attitude.roll,
///					state.attitude.yaw, sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z);
			}
			powerDistribution(&control);
		} else {
			powerStop();
			inTheAir = false;
			if (!(tick % 500)) { //  ***************************************************************
				returnData.a = 0;
				returnData.b = 0;
				returnData.c = 0;
				returnData.d = 0;
				returnData.e = 0;
				returnData.f = 0;
				returnData.fa = 0;
				returnData.fb = 0;
				returnData.fc = setpoint.position.z;
				returnData.fd = 0;
				returnData.fe = 0,
				returnData.ff = 0;
				xQueueSend(espnow_queueTx, &returnData, 0);
			}
		}

		tick++;
	}
}
