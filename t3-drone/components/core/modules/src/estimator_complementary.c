

#
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "stabilizer.h"
#include "estimator_complementary.h"
#include "sensfusion6.h"
#include "position_estimator.h"
#include "sensor_spl06.h"
#include "sensor_mpu6050.h"
#include "stabilizer_types.h"
#include "static_mem.h"
#include "estimator_utils.h"
#include "kalman_core_v2.h"
#include "flowdeck_v1v2.h"
#include "esp_log.h"
#include "mcc5983ma.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

#define MIN_HEIGHT_FOR_FLOW 0.3

//static bool latestTofMeasurement(tofMeasurement_t *tofMeasurement);
float dx, dy;
float stDev = 0.025;
flowQ_t delta;
//uint32_t lastKalman;
uint32_t lastBaro;
bool newBaro = false;

static void calculateFlowAndHeight(float *dx, float *dy, state_t *state, flowQ_t dxy, sensorData_t *sensor, float dt);

// Measurements of TOF from laser sensor
//#define TOF_QUEUE_LENGTH (1)
//static xQueueHandle tofDataQueue;
//STATIC_MEM_QUEUE_ALLOC(tofDataQueue, TOF_QUEUE_LENGTH, sizeof(tofMeasurement_t));

void estimatorComplementaryInit(void) {
//	tofDataQueue = STATIC_MEM_QUEUE_CREATE(tofDataQueue);
	sensfusion6Init();
	//kalmanCoreInit2();
}

bool estimatorComplementaryTest(void) {
	bool pass = true;

	pass &= sensfusion6Test();

	return pass;
}

void estimatorComplementary(state_t *state, sensorData_t *sensorData, control_t *control, const uint32_t tick) {
	sensorsMpu6050Acquire(sensorData, tick); // Read sensors at full rate (1000Hz)
	sensorReadMag(&sensorData->mag);
	/*if (RATE_DO_EXECUTE(5, tick)) {
		ESP_LOGI(" ", "%d ", sensorData->mag);
	}*/
	if (sensorReadBaro(&sensorData->asl)) {
		newBaro = true;
	}
	if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
		//if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
		sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z, sensorData->acc.x, sensorData->acc.y,
				sensorData->acc.z,
				ATTITUDE_UPDATE_DT);
// roll=gyro.x, pitch=-gyro.y, yaw otocen oproti obr , yaw = z
		// acc.x , dopredu je + acc
		// acc.y , doprava je - acc
		// Save attitude, adjusted for the legacy CF2 body coordinate system
		sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);
//		ESP_LOGI(" ", "asl %f, gyro %f %f %f, acc %f %f %f, att %f %f %f", sensorData->asl, sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
//		        sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
//				state->attitude.roll, state->attitude.pitch, state->attitude.yaw);

		// Save quaternion, hopefully one day this could be used in a better controller.
		// Note that this is not adjusted for the legacy coordinate system
		sensfusion6GetQuaternion(&state->attitudeQuaternion.x, &state->attitudeQuaternion.y, &state->attitudeQuaternion.z,
				&state->attitudeQuaternion.w);

		calculateRotationMatrix(state);  // calculate rotation matrix from quaternion
		calculateRotation(&state->acc.x, &state->acc.y, &state->acc.z, sensorData->acc.x, sensorData->acc.y, sensorData->acc.z, state);

		// sensor na flow funguje od 8cm, nastaveno na 0.1m
		if (sensorData->asl - state->baroReferenceHeight > MIN_HEIGHT_FOR_FLOW) {

			// pocitej jen pokud jsou data, flow ma nastaveno 100Hz, nutno otestovat jaka frequence bude idealni
			if (sensorReadFlow(&delta)) {
				// je pohyb?
				if (delta.dx != 0 || delta.dy != 0) {
					float dt = (float) (tick - lastKalman) ; // in ms
					if (dt < 20) {
						dt /=1000; // to sec
						// x je +doprava
						// y je +zpet, proto
						delta.dy = -delta.dy;
						// prepocti flow pres vysku
						calculateFlowAndHeight(&dx, &dy, state, delta, sensorData, dt); // otaci souradnice
						//ESP_LOGI(" ", "%f %f %f %f", dx, dy, state->baroReferenceHeight, sensorData->asl);
						//prida  X a Y pozici a rychlost
						kalman2(state, dx, dy, stDev, dt);
						ESP_LOGI(" ", "%f %f %f %f %d %d", dx, dy, state->position.x,state->position.y,tick,lastKalman);
					//	state->position.x = delta.dx;
					//	state->position.y = delta.dy;
					//	state->velocity.x = dx;
					//	state->velocity.y = dy;
					}
					lastKalman = tick;
				}
			}
		}

		// uz je spocitany
		//state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x, sensorData->acc.y, sensorData->acc.z);
		// neni nutny, spocita se nize v positionEstimate(...
		//positionUpdateVelocity(state->acc.z - state->zGravityBase, ATTITUDE_UPDATE_DT);
	}

	//vysku betreba pocitat casteji protoze baro dava data 128Hz
	// Az pridame laser mereni pak zmenit
	//if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) {
	if (newBaro) {

		//ESP_LOGI(" ", "%f ", sensorData->asl);
		//doplni  Z pozici a rychlost
		positionEstimate(state, sensorData, (float) (tick - lastBaro) / 1000, tick);
		lastBaro = tick;
		//ESP_LOGI(" ", "asl %f %f %f", sensorData->asl, state->position.z,state->velocity.z);
		newBaro = false;
	}
}

// prepocet pohybu ze sensoru na pixely ctene z vyscky 1m a prevod z int na float
static void calculateFlowAndHeight(float *dx, float *dy, state_t *state, flowQ_t dxy, sensorData_t *sensor, float dt) {
	// pmw9310 spc: 30x30pixels, FOV 42� - 1pixel  move v 1m vyska je posun 0.0256m
	static const float pmw3901multi = 0.0256;
	//posun v m = posun v pixelech * multi * vyska v m
	// flow vraci doprava +x a doprevu +y, nutno prohodit
	float x = dxy.dy;
	float y = dxy.dx;
	x = (-(sensor->gyro.y * dt * 30) / 42 + x) * pmw3901multi * (sensor->asl - state->baroReferenceHeight);
	y = ((sensor->gyro.x * dt * 30) / 42 + y) * pmw3901multi * (sensor->asl - state->baroReferenceHeight);
	float yawRad = state->attitude.yaw * DEG_TO_RAD;

	//  mela by se pouzit rotacni matice
	*dx = (x * cosf(yawRad)) - (y * sinf(yawRad));
	*dy = (-x * sinf(yawRad)) + (y * cosf(yawRad));
	//ESP_LOGI(" ", "%d %d %f %f %f %f %f %f %f", dxy.dx, dxy.dy, x, y, *dx, *dy, sensor->asl - state->baroReferenceHeight, state->attitude.roll, state->attitude.pitch);
}
