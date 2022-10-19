
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "commander.h"
#include "cf_math.h"
#include "utils.h"
#include "static_mem.h"
#include "stabilizer_types.h"
#include "espnow.h"

static bool isInit;
//const static setpoint_t nullSetpoint;
static setpoint_t tempSetpoint;
//static state_t lastState;
//const static int priorityDisable = COMMANDER_PRIORITY_DISABLE;

static uint32_t lastUpdate;
//static bool enableHighLevel = false;

//static QueueHandle_t setpointQueue;
//STATIC_MEM_QUEUE_ALLOC(setpointQueue, 1, sizeof(setpoint_t));
//static QueueHandle_t priorityQueue;
//STATIC_MEM_QUEUE_ALLOC(priorityQueue, 1, sizeof(int));

/* Public functions */
void commanderInit(void) {
//	setpointQueue = STATIC_MEM_QUEUE_CREATE(setpointQueue);
	//ASSERT(setpointQueue);
///	xQueueSend(setpointQueue, &nullSetpoint, 0);

//	priorityQueue = STATIC_MEM_QUEUE_CREATE(priorityQueue);
	//ASSERT(priorityQueue);
//	xQueueSend(priorityQueue, &priorityDisable, 0);

//  crtpCommanderInit();
//  crtpCommanderHighLevelInit();
	lastUpdate = xTaskGetTickCount();

	isInit = true;
}
/*
 void commanderSetSetpoint(setpoint_t *setpoint, int priority) {
 int currentPriority;

 const BaseType_t peekResult = xQueuePeek(priorityQueue, &currentPriority,
 0);
 //ASSERT(peekResult == pdTRUE);

 if (priority >= currentPriority) {
 setpoint->timestamp = xTaskGetTickCount();
 // This is a potential race but without effect on functionality
 xQueueOverwrite(setpointQueue, setpoint);
 xQueueOverwrite(priorityQueue, &priority);
 // Send the high-level planner to idle so it will forget its current state
 // and start over if we switch from low-level to high-level in the future.
 //    crtpCommanderHighLevelStop();
 }
 }

 void commanderNotifySetpointsStop(int remainValidMillisecs) {
 uint32_t currentTime = xTaskGetTickCount();
 int timeSetback = MIN(
 COMMANDER_WDT_TIMEOUT_SHUTDOWN - M2T(remainValidMillisecs),
 currentTime);
 xQueuePeek(setpointQueue, &tempSetpoint, 0);
 tempSetpoint.timestamp = currentTime - timeSetback;
 xQueueOverwrite(setpointQueue, &tempSetpoint);
 // crtpCommanderHighLevelTellState(&lastState);
 }*/

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state) {

	myData_message myData;

	xQueuePeek(espnow_queueRx, &myData, 0);

	/*	 setpoint->mode.x = modeDisable;
	 setpoint->mode.y = modeDisable;
	 setpoint->mode.z = modeDisable;
	 setpoint->velocity.x = 0;
	 setpoint->velocity.y = 0;
	 setpoint->mode.roll = modeAbs;
	 setpoint->mode.pitch = modeAbs;
	 setpoint->mode.yaw = modeVelocity;
	 setpoint->attitude.roll = (myData.pitch - 1500) / 50.0; // to degrees, max �
	 //	setpoint->attitude.roll = 0;
	 setpoint->attitudeRate.roll = 0;
	 setpoint->attitude.pitch = -(myData.roll - 1500) / 50.0;
	 //	setpoint->attitude.pitch = 0;
	 setpoint->attitudeRate.pitch = 0;
	 setpoint->attitude.yaw = 0;
	 setpoint->attitudeRate.yaw = -(myData.yaw - 1500) / 50.0;
	 //	setpoint->attitudeRate.yaw = 0;
	 setpoint->velocity_body = false;

	 setpoint->thrust = ((myData.throttle-1000) * 40); // rescale!!!!!!!!!!!!!!
	 */

	if (myData.flyDron)
		setpoint->flyDron = true;
	else
		setpoint->flyDron = false;
	setpoint->attitude.yaw = 0;	//state->attitude.yaw;
	setpoint->position.z = (myData.throttle - 1000.0) / 200.0; //0.5; //vyska pul metru
	setpoint->position.x = 0;
	setpoint->position.y = 0;

	/*  xQueuePeek(setpointQueue, setpoint, 0);
	 lastUpdate = setpoint->timestamp;
	 uint32_t currentTime = xTaskGetTickCount();

	 if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
	 if (enableHighLevel) {
	 crtpCommanderHighLevelGetSetpoint(setpoint, state);
	 }
	 if (!enableHighLevel || crtpCommanderHighLevelIsStopped()) {
	 memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
	 }
	 } else if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_STABILIZE) {
	 xQueueOverwrite(priorityQueue, &priorityDisable);
	 // Leveling ...
	 setpoint->mode.x = modeDisable;
	 setpoint->mode.y = modeDisable;
	 setpoint->mode.roll = modeAbs;
	 setpoint->mode.pitch = modeAbs;
	 setpoint->mode.yaw = modeVelocity;
	 setpoint->attitude.roll = 0;
	 setpoint->attitude.pitch = 0;
	 setpoint->attitudeRate.yaw = 0;
	 // Keep Z as it is
	 //  }*/
}
/*
 bool commanderTest(void) {
 return isInit;
 }

 uint32_t commanderGetInactivityTime(void) {
 return xTaskGetTickCount() - lastUpdate;
 }

 int commanderGetActivePriority(void) {
 int priority;

 const BaseType_t peekResult = xQueuePeek(priorityQueue, &priority, 0);
 //ASSERT(peekResult == pdTRUE);

 return priority;
 }
 */
