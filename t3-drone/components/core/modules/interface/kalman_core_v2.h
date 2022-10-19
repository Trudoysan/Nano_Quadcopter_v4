

#ifndef COMPONENTS_CORE_MODULES_INTERFACE_KALMAN_CORE_V2_H_
#define COMPONENTS_CORE_MODULES_INTERFACE_KALMAN_CORE_V2_H_



#include "cf_math.h"
#include "stabilizer_types.h"

// Indexes to access the quad's state, stored as a column vector
typedef enum
{
  KC_STATE_X, KC_STATE_Y, KC_STATE_PX, KC_STATE_PY, KC_STATE_DIM
} kalmanCoreStateIdx_t2;


// The data used by the kalman core implementation.
typedef struct {
  /**
   * Quadrocopter State
   *
   * The internally-estimated state is:
   * - X, Y: the quad's position in the global frame
   * - PX, PY: the quad's velocity in its body frame
   */
  float S2[KC_STATE_DIM];

  // The quad's attitude as a quaternion (w,x,y,z)
  // We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
  // while also being robust against singularities (in comparison to euler angles)
  //float q2[4];

  // The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
  //float R2[3][3];

  // The covariance matrix
  float P2[KC_STATE_DIM];
  //xtensa_matrix_instance_f32 Pm2;

  // Indicates that the internal state is corrupt and should be reset
  //bool resetEstimation2;

//  float baroReferenceHeight;
} kalmanCoreData_t2;

void kalmanCoreInit2(state_t *state);
void kalman2(state_t *state, float dx, float dy, float stDev, float dt);

#endif /* COMPONENTS_CORE_MODULES_INTERFACE_KALMAN_CORE_V2_H_ */
