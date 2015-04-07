// Parameters and functions definitions for a Kalman estimator
// used to denoise a noisy signal.

// **********************************************************************
// KALMAN STRUCTURE DEFINITION
typedef struct
{
  float q;      // Process noise estimated covariance
  float r;      // Measurement noise estimated covariance
  float x_est;  // Estimated signal
  float p;      // Error estimated covariance
  float k;      // Kalman gain
} kalman_state;
// **********************************************************************

// **********************************************************************
// KALMAN FUNCTIONS DECLARATION
// ************************************************************
// Function kalman_init
// Action : Initializes the kalman estimator
// Arguments :
//      q :             Process noise estimated covariance
//      r :             Measurement noise estimated covariance
//      p :             Error estimated covariance
//      initialValue :  Kalman gain
// Return value : Kalman estimator parameters
kalman_state kalman_init(float q, float r, float p, float initialValue);
// ************************************************************
// Function kalman_update
// Action : Updates the Kalman estimator parameters
// Arguments : 
//      state : Kalman structure
//      signal : Measured signal
// Return value : Estimated signal
float kalman_update(kalman_state* state, float signal);
// ************************************************************
