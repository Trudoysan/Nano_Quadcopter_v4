

/*
 #include <math.h>

 #include "freertos/FreeRTOS.h"
 #include "freertos/semphr.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/projdefs.h"
 #include "esp_timer.h"
 #include "driver/gpio.h"
 #include "nvicconf.h"
 #include "filter.h"

 */
#include "sensor_mpu6050.h"
#include "imu_types.h"
#include "utils.h"
#include "i2cdev.h"
#include "mpu6050.h"
#include "static_mem.h"
#include "esp_log.h"
#include "filter.h"
#include "system.h"
#include "usec_time.h"
#include "config.h"
#include "led.h"
#include "ledseq.h"
#include "stabilizer_types.h"

#define SENSORS_GYRO_FS_CFG MPU6050_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG MPU6050_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG MPU6050_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG MPU6050_G_PER_LSB_16
#define SENSORS_ACC_SCALE_SAMPLES 200
#define GPIO_INTA_MPU6050_IO CONFIG_MPU_PIN_INT
#define SENSORS_MPU6050_BUFF_LEN 14
#define GYRO_NBR_OF_AXES 3
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(1 * 1000)

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE 5000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

#define ESP_INTR_FLAG_DEFAULT 0

#define PITCH_CALIB (CONFIG_PITCH_CALIB*1.0/100)
#define ROLL_CALIB (CONFIG_ROLL_CALIB*1.0/100)

typedef struct {
	Axis3f bias;
	Axis3f variance;
	Axis3f mean;
	bool isBiasValueFound;
	bool isBufferFilled;
	Axis3i16 *bufHead;
	Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));

static xSemaphoreHandle sensorsDataReady;
static xSemaphoreHandle dataReady;

static bool isInit = false;

static sensorData_t sensorData;

static volatile uint64_t imuIntTimestamp;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;

static BiasObj gyroBiasRunning;

static Axis3f gyroBias;
static bool gyroBiasFound = false;
static float accScaleSum = 0;

static float accScale = 1;

// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ 80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);

// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

// This buffer needs to hold data from all sensors
static uint8_t buffer[SENSORS_MPU6050_BUFF_LEN] = { 0 };

static void processAccGyroMeasurements(const uint8_t *buffer);

static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);

static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);

static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

bool sensorsMpu6050ReadGyro(Axis3f *gyro) {
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsMpu6050ReadAcc(Axis3f *acc) {
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

void sensorsMpu6050Acquire(sensorData_t *sensors, const uint32_t tick) {
	sensorsMpu6050ReadGyro(&sensors->gyro);
	sensorsMpu6050ReadAcc(&sensors->acc);

	sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsMpu6050Calibrated() {
	return gyroBiasFound;
}

static void sensorsTask(void *param) {
	//TODO:

	systemWaitStart();
	ESP_LOGI("sensorsTask", "Started");
	vTaskDelay(M2T(200));

	while (1) {

		// mpu6050 interrupt trigger: data is ready to be read
		if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {

			sensorData.interruptTimestamp = imuIntTimestamp;

			// sensors step 1-read data from I2C
			uint8_t dataLen = (uint8_t) (SENSORS_MPU6050_BUFF_LEN);
			i2cdevReadReg8(I2C0_DEV, MPU6050_ADDRESS_AD0_LOW,
			MPU6050_RA_ACCEL_XOUT_H, dataLen, buffer);

			// sensors step 2-process the respective data
			processAccGyroMeasurements(&(buffer[0]));
			// sensors step 3- queue sensors data  on the output queues
			xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
			xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
			// sensors step 4- Unlock stabilizer task
			xSemaphoreGive(dataReady);
		}
	}
}

void sensorsMpu6050WaitDataReady(void) {
	xSemaphoreTake(dataReady, portMAX_DELAY);
}

void processAccGyroMeasurements(const uint8_t *buffer) {
	//  Note the ordering to correct the rotated 90º IMU coordinate system

	Axis3f accScaled;

	// sensors step 2.1 read from buffer
	accelRaw.x = (((int16_t) buffer[0]) << 8) | buffer[1];
	accelRaw.y = (((int16_t) buffer[2]) << 8) | buffer[3];
	accelRaw.z = (((int16_t) buffer[4]) << 8) | buffer[5];
	gyroRaw.x = (((int16_t) buffer[8]) << 8) | buffer[9];
	gyroRaw.y = (((int16_t) buffer[10]) << 8) | buffer[11];
	gyroRaw.z = (((int16_t) buffer[12]) << 8) | buffer[13];

	// sensors step 2.2 Calculates the gyro bias first when the  variance is below threshold
	gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);

	//sensors step 2.3 Calculates the acc scale when platform is steady
	if (gyroBiasFound) {
		processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
	}

	// sensors step 2.4 convert  digtal value to physical angle
	sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG; //rotace - pred zavorkou
	sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;

	// sensors step 2.5 low pass filter
	applyAxis3fLpf((lpf2pData*) (&gyroLpf), &sensorData.gyro);

	accScaled.x = (accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale; //rotace - pred zavorkou
	accScaled.y = (accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
	accScaled.z = (accelRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;

	// sensors step 2.6 Compensate for a miss-aligned accelerometer.
	sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
	applyAxis3fLpf((lpf2pData*) (&accLpf), &sensorData.acc);
}

static void sensorsDeviceInit(void) {
	// Wait for sensors to startup
	while (xTaskGetTickCount() < 2000) {
		vTaskDelay(M2T(50));
	};

	//i2cdevInit(I2C0_DEV); uz udelano
	mpu6050Init(I2C0_DEV);

	if (mpu6050TestConnection() == true) {
		ESP_LOGI("sensorsDeviceInit", "MPU6050 I2C connection [OK].");
	} else {
		ESP_LOGW("sensorsDeviceInit", "MPU6050 I2C connection [FAIL].");
	}

	mpu6050Reset();
	vTaskDelay(M2T(50));
	// Activate mpu6050
	mpu6050SetSleepEnabled(false);
	// Delay until registers are reset
	vTaskDelay(M2T(100));
	// Set x-axis gyro as clock source
	mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
	// Delay until clock is set and stable
	vTaskDelay(M2T(200));
	// Enable temp sensor
	mpu6050SetTempSensorEnabled(true);
	// Disable interrupts
	mpu6050SetIntEnabled(false);
	// Connect the MAG and BARO to the main I2C bus
	// ************** mpu6050SetI2CBypassEnabled(true);
	mpu6050SetI2CBypassEnabled(false);
	// Set gyro full scale range
	mpu6050SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);
	// Set accelerometer full scale range
	mpu6050SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);

	// To low DLPF bandwidth might cause instability and decrease agility
	// but it works well for handling vibrations and unbalanced propellers
	// Set output rate (1): 1000 / (1 + 0) = 1000Hz
	mpu6050SetRate(0);
	mpu6050SetDLPFMode(MPU6050_DLPF_BW_42);
	// Init second order filer for accelerometer
	for (uint8_t i = 0; i < 3; i++) {
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
	}

	cosPitch = cosf(PITCH_CALIB * (float) M_PI / 180);
	sinPitch = sinf(PITCH_CALIB * (float) M_PI / 180);
	cosRoll = cosf(ROLL_CALIB * (float) M_PI / 180);
	sinRoll = sinf(ROLL_CALIB * (float) M_PI / 180);
	ESP_LOGI("sensorsDeviceInit", "pitch_calib = %f,roll_calib = %f", PITCH_CALIB, ROLL_CALIB);

}

static void sensorsTaskInit(void) {
	accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
	gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);

	STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);

}

static void IRAM_ATTR sensors_inta_isr_handler(void *arg) {

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	imuIntTimestamp = usecTimestamp(); //This function returns the number of microseconds since esp_timer was initialized
	xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR();
	}
}

static void sensorsInterruptInit(void) {
	gpio_config_t io_conf;
	//interrupt of rising edge
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	//bit mask of the pins
	io_conf.pin_bit_mask = (1ULL << GPIO_INTA_MPU6050_IO);
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	sensorsDataReady = xSemaphoreCreateBinary();
	dataReady = xSemaphoreCreateBinary();
	gpio_config(&io_conf);
	//install gpio isr service
	//portDISABLE_INTERRUPTS();
	gpio_set_intr_type(GPIO_INTA_MPU6050_IO, GPIO_INTR_POSEDGE);
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INTA_MPU6050_IO, sensors_inta_isr_handler, (void*) GPIO_INTA_MPU6050_IO);
	//portENABLE_INTERRUPTS();

	//   FSYNC "shall not be floating, must be set high or low by the MCU"

	mpu6050SetInterruptMode(0);                       // active high
	mpu6050SetInterruptDrive(0);                      // push pull
	mpu6050SetInterruptLatch(0);                      // latched until clear
	mpu6050SetInterruptLatchClear(1);        // cleared on any register read

	mpu6050SetIntDataReadyEnabled(true);

}

void sensorMpu6050Init(void) {
	if (isInit) {
		return;
	}
	sensorsBiasObjInit(&gyroBiasRunning);
	sensorsDeviceInit();
	sensorsInterruptInit();
	sensorsTaskInit();
	isInit = true;
}

static bool processAccScale(int16_t ax, int16_t ay, int16_t az) {
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound) {
		accScaleSum += sqrtf(
				powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}

	return accBiasFound;
}



// Calculates the bias first when the gyro variance is below threshold. Requires a buffer
// but calibrates platform first when it is stable.

static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut) {
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound) {
		sensorsFindBiasValue(&gyroBiasRunning);

		if (gyroBiasRunning.isBiasValueFound) {
			ledseqRun(ERR_LED, seq_calibrated);
			ESP_LOGI("processGyroBias", "Bias Found!");
		}
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}


static void sensorsBiasObjInit(BiasObj *bias) {
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}

// Calculates the variance and mean for the bias buffer.

static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut) {
	uint32_t i;
	int64_t sum[GYRO_NBR_OF_AXES] = { 0 };
	int64_t sumSq[GYRO_NBR_OF_AXES] = { 0 };

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumSq[0] - ((int64_t) sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumSq[1] - ((int64_t) sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumSq[2] - ((int64_t) sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float) sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float) sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float) sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}


// Adds a new value to the variance buffer and if it is full
// replaces the oldest one. Thus a circular buffer.
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z) {
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}

//  Checks if the variances is below the predefined thresholds.
//  The bias value should have been added before calling this.
//  @param bias  The bias object

static bool sensorsFindBiasValue(BiasObj *bias) {
	static int32_t varianceSampleTime;
	bool foundBias = false;

	if (bias->isBufferFilled) {
		sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

		if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X && bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y
				&& bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z && (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
			varianceSampleTime = xTaskGetTickCount();
			bias->bias.x = bias->mean.x;
			bias->bias.y = bias->mean.y;
			bias->bias.z = bias->mean.z;
			foundBias = true;
			bias->isBiasValueFound = true;
		}
	}

	return foundBias;
}

static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out) {
	Axis3f rx;
	Axis3f ry;

	// Rotate around x-axis
	rx.x = in->x;
	rx.y = in->y * cosRoll - in->z * sinRoll;
	rx.z = in->y * sinRoll + in->z * cosRoll;

	// Rotate around y-axis
	ry.x = rx.x * cosPitch - rx.z * sinPitch;
	ry.y = rx.y;
	ry.z = -rx.x * sinPitch + rx.z * cosPitch;

	out->x = ry.x;
	out->y = ry.y;
	out->z = ry.z;
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in) {
	for (uint8_t i = 0; i < 3; i++) {
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}
