
#ifndef STABILIZER_H_
#define STABILIZER_H_

#include <stdbool.h>
#include <stdint.h>

bool emergStop;

/**
 * Initialize the stabilizer subsystem and launch the stabilizer loop task.
 * The stabilizer loop task will wait on systemWaitStart() before running.
 */
void stabilizerInit(void/*StateEstimatorType estimator*/);

/**
 * Test the stabilizer subsystem. Calls test for all the stabilizer related
 * sensors.
 * @return True if all test has passed. False otherwise.
 */
bool stabilizerTest(void);

/**
 * Enable emergency stop, will shut-off energy to the motors.
 */
void stabilizerSetEmergencyStop();

/**
 * Disable emergency stop, will enable energy to the motors.
 */
void stabilizerResetEmergencyStop();

/**
 * Restart the countdown until emergercy stop will be enabled.
 *
 * @param timeout Timeout in stabilizer loop tick. The stabilizer loop rate is
 *                RATE_MAIN_LOOP.
 */
void stabilizerSetEmergencyStopTimeout(int timeout);


#endif /* STABILIZER_H_ */
