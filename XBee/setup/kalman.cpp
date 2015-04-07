#include "kalman.h"

// **********************************************************************
// KALMAN FUNCTIONS DEFINITION
// ************************************************************
// Function kalman_init
// Action : Initializes the kalman estimator
// Arguments :
//      q :             Process noise estimated covariance
//      r :             Measurement noise estimated covariance
//      p :             Error estimated covariance
//      initialValue :  Kalman gain
// Return value : Kalman estimator parameters
kalman_state kalman_init(float q, float r, float p, float initialValue)
{
  kalman_state result;

  result.q = q;
  result.r = r;
  result.p = p;
  result.x_est = initialValue;

  return (result);
}
// ************************************************************
// Function kalman_update
// Action : Updates the Kalman estimator parameters
// Arguments :
//      state : Kalman structure
//      signal : Measured signal
// Return value : Estimated signal
float kalman_update(kalman_state* state, float signal)
{
  // Time Update phase
  state->p = state->p + state->q;

  // Measurement Update phase
  state->k = state->p / (state->p + state->r);
  state->x_est = state->x_est + state->k * (signal - state->x_est);
  state->p = (1 - state->k) * state->p;

  return (state->x_est);
}
// ************************************************************
