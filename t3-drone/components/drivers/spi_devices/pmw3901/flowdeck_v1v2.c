/*
 *
 *
 * LPS node firmware.
 *
 * Copyright 2017, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* flowdeck.c: Flow deck driver */
#include "static_mem.h"
//#include <stdlib.h>


#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pmw3901.h"
#include "system.h"
#include "utils.h"
#include "config.h"
#include "stabilizer_types.h"
//#include "cf_math.h"


#define AVERAGE_HISTORY_LENGTH 4
#define LP_CONSTANT 0.8f
//#define USE_LP_FILTER
//#define USE_MA_SMOOTHING

#if defined(USE_MA_SMOOTHING)
static struct {
    float32_t averageX[AVERAGE_HISTORY_LENGTH];
    float32_t averageY[AVERAGE_HISTORY_LENGTH];
    size_t ptr;
} pixelAverages;
#endif

float dpixelx_previous = 0;
float dpixely_previous = 0;

static bool isInit1 = false;
static bool isInit2 = false;

static flowQ_t delta;

static xQueueHandle deltaDataQueue;
STATIC_MEM_QUEUE_ALLOC(deltaDataQueue, 1, sizeof(flowQ_t));


//motionBurst_t currentMotion;

#define NCS_PIN CONFIG_SPI_PIN_CS0

/*void readFlow(int16_t*dX, int16_t *dY) {
	*dX = deltaX;
	*dY = deltaY;
}*/

void flowdeckTask(void *arg) {

	systemWaitStart();
	TickType_t osTick = xTaskGetTickCount();

	while (1) {
// if task watchdog triggered,flow frequency should set lower
		//vTaskDelay(10); //10
		vTaskDelayUntil(&osTick, F2T(100));
		readMotionCount(&delta.dx, &delta.dy, NCS_PIN);
		xQueueOverwrite(deltaDataQueue, &delta);
		//pmw3901ReadMotion(NCS_PIN, &currentMotion);
		/*
		 ESP_LOGI("flowdeckTask", "%d %d", currentMotion.deltaX, currentMotion.deltaY);

		 // Flip motion information to comply with sensor mounting
		 // (might need to be changed if mounted differently)
		 int16_t accpx = -currentMotion.deltaY;
		 int16_t accpy = -currentMotion.deltaX;

		 // Outlier removal
		 if (abs(accpx) < OULIER_LIMIT && abs(accpy) < OULIER_LIMIT) {

		 // Form flow measurement struct and push into the EKF
		 flowMeasurement_t flowData;
		 flowData.stdDevX = 0.25;    // [pixels] should perhaps be made larger?
		 flowData.stdDevY = 0.25;    // [pixels] should perhaps be made larger?

		 // if task watchdog triggered,flow frequency should set lower
		 flowData.dt = 0.01;


		 #if defined(USE_MA_SMOOTHING)
		 // Use MA Smoothing
		 pixelAverages.averageX[pixelAverages.ptr] = (float32_t)accpx;
		 pixelAverages.averageY[pixelAverages.ptr] = (float32_t)accpy;

		 float32_t meanX;
		 float32_t meanY;

		 xtensa_mean_f32(pixelAverages.averageX, AVERAGE_HISTORY_LENGTH, &meanX);
		 xtensa_mean_f32(pixelAverages.averageY, AVERAGE_HISTORY_LENGTH, &meanY);

		 pixelAverages.ptr = (pixelAverages.ptr + 1) % AVERAGE_HISTORY_LENGTH;

		 flowData.dpixelx = (float)meanX;   // [pixels]
		 flowData.dpixely = (float)meanY;   // [pixels]
		 #elif defined(USE_LP_FILTER)
		 // Use LP filter measurements
		 flowData.dpixelx = LP_CONSTANT * dpixelx_previous + (1.0f - LP_CONSTANT) * (float)accpx;
		 flowData.dpixely = LP_CONSTANT * dpixely_previous + (1.0f - LP_CONSTANT) * (float)accpy;
		 dpixelx_previous = flowData.dpixelx;
		 dpixely_previous = flowData.dpixely;
		 #else
		 // Use raw measurements
		 flowData.dpixelx = (float)accpx;
		 flowData.dpixely = (float)accpy;
		 #endif

		 // Push measurements into the estimator
		 //if (!useFlowDisabled) {
		 //estimatorKalmanEnqueueFlow(&flowData);
		 //  estimatorEnqueueFlow(&flowData);
		 }
		 } else {
		 outlierCount++;
		 }*/
	}
}

// static void flowdeck1Init()
// {
//   if (isInit1 || isInit2) {
//     return;
//   }

//   // Initialize the VL53L0 sensor using the zRanger deck driver
//   const DeckDriver *zRanger = deckFindDriverByName("bcZRanger");
//   zRanger->init(NULL);

//   if (pmw3901Init(NCS_PIN))
//   {
//     xTaskCreate(flowdeckTask, FLOW_TASK_NAME, FLOW_TASK_STACKSIZE, NULL,
//                 FLOW_TASK_PRI, NULL);

//     isInit1 = true;
//   }
// }

// static bool flowdeck1Test()
// {
//   if (!isInit1) {
//     DEBUG_PRINTD("Error while initializing the PMW3901 sensor\n");
//   }

//   // Test the VL53L0 driver
//   const DeckDriver *zRanger = deckFindDriverByName("bcZRanger");

//   return zRanger->test();
// }

bool sensorReadFlow(flowQ_t *delta) {
	return (pdTRUE == xQueueReceive(deltaDataQueue, delta, 0));

}

void flowdeck2Init(void) {
	if (isInit1 || isInit2) {
		return;
	}

	// Initialize the VL53L1 sensor using the zRanger deck driver
	// const DeckDriver *zRanger = deckFindDriverByName("bcZRanger2");
	// zRanger->init(NULL);

	if (pmw3901Init(NCS_PIN)) {
		xTaskCreate(flowdeckTask, FLOW_TASK_NAME, FLOW_TASK_STACKSIZE, NULL, FLOW_TASK_PRI, NULL);
		deltaDataQueue = STATIC_MEM_QUEUE_CREATE(deltaDataQueue);
		isInit2 = true;
	}
}
