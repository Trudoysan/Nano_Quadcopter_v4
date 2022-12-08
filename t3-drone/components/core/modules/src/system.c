

//#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "config.h"
#include "static_mem.h"
#include "led.h"
#include "utils.h"
#include "espnow.h"
#include "ledseq.h"
#include "sensor_mpu6050.h"
#include "i2cdev.h"
#include "stabilizer.h"
#include "system.h"
#include "sensor_spl06.h"
#include "flowdeck_v1v2.h"
#include "mcc5983ma.h"
 #include "commander.h"
//#include "adc_esp32.h"


static bool isInit;

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);


xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

static void systemTask(void *arg);

void systemLaunch(void) {
	STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}


void systemInit(void) {
	if (isInit)
		return;

	canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
	xSemaphoreTake(canStartMutex, portMAX_DELAY);

	ledseqInit();
	//	adcInit();
	isInit = true;
}

void systemTask(void *arg) {
//	bool pass = true;

	ledInit();
	ledSetRed(true);
	wifi_init();
	ESP_ERROR_CHECK(espnow_init());
	vTaskDelay(M2T(500));

	systemInit();
	ledseqRun(ERR_LED, seq_alive);
	i2cdevInit(I2C0_DEV);
	sensorMpu6050Init();
	sensor_spl06Init();
	mcc5983Init();
	flowdeck2Init();
	commanderInit();

	stabilizerInit();

	systemStart();
	ESP_LOGI("systemTask", "System started, Free heap: %d bytes", xPortGetFreeHeapSize());

	//workerLoop();


	while (1) {
		ESP_LOGI("systemTask", "while(1) Delay");
		vTaskDelay(portMAX_DELAY);
	}
}


void systemStart() {
	xSemaphoreGive(canStartMutex);
}

void systemWaitStart(void) {

	while (!isInit)
		vTaskDelay(2);

	xSemaphoreTake(canStartMutex, portMAX_DELAY);
	xSemaphoreGive(canStartMutex);
}

