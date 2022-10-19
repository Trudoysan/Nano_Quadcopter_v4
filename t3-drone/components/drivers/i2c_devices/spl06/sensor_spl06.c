
#include "config.h"
#include "static_mem.h"
#include "sensor_spl06.h"
#include "spl06.h"
#include "stabilizer_types.h"
#include "utils.h"
#include "system.h"
#include "cf_math.h"
#include "stdbool.h"
/*
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"


 #include "esp_log.h"
 #include "i2cdev.h"
 */
#define BARO_FREQ 25

static bool isInit;

static void calculateSD(void);

void spl06Task(void *arg);

static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(float));

void sensor_spl06Init(void) {
	if (isInit)
		return;

	if (spl06Init(I2C0_DEV) == false) {
		useBaroUpdate = false;
		return;
	}

	barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

	calculateSD();
	useBaroUpdate = true;
	xTaskCreate(spl06Task, SPL06_TASK_NAME, SPL06_TASK_STACKSIZE, NULL,
	SPL06_TASK_PRI, NULL);

	isInit = true;
}

void spl06Task(void *arg) {

	systemWaitStart();
	float asl;

	TickType_t osTick = xTaskGetTickCount();
	while (1) {
		vTaskDelayUntil(&osTick, F2T(4));
		asl = spl06Read();
		xQueueOverwrite(barometerDataQueue, &asl);
	}
}

bool sensorReadBaro(float *asl) {
	return (pdTRUE == xQueueReceive(barometerDataQueue, asl, 0));
}

static void calculateSD(void) {
	float sum = 0.0, mean, SD = 0.0;
	int i;
	int num_aslSD = 10;
	float aslSD[num_aslSD];

	TickType_t osTick = xTaskGetTickCount();
	for (i = 0; i < num_aslSD; ++i) {
		vTaskDelayUntil(&osTick, F2T(BARO_FREQ));
		aslSD[i] = spl06Read();
		sum += aslSD[i];
	}
	mean = sum / num_aslSD;
	for (i = 0; i < num_aslSD; ++i)
		SD += pow(aslSD[i] - mean, 2);
	baro_stddev = sqrt(SD / num_aslSD);
}

