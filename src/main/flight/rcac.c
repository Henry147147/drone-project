#include "platform.h"
#include "pid.h"
#include "rcac.h"
#include "build/debug.h"
#include <math.h>
#include <stdlib.h>

static void InitializeWindowBuffers(const RCAC_hyperparameters_t *hyperParams, RCAC_internal_state_t *state);
static void InitalizeHBuffer(RCAC_internal_state_t *internalState, const RCAC_hyperparameters_t *hyperParams);
static void UpdateHBuffers(const RCAC_input_output_t *inputOutput, const RCAC_hyperparameters_t *hyperparams, RCAC_internal_state_t *state);
static void buildRegressor(RCAC_internal_state_t *state, int k, uint8_t ltheta);
static void updateWindowBuffer(RCAC_internal_state_t *state, const RCAC_hyperparameters_t *hyperparams, float u_in, float z_in) __attribute__((unused));
static void PrintBuffers(const RCAC_input_output_t *inputOutput, const RCAC_hyperparameters_t *hyperParams, const RCAC_internal_state_t *state);
static float *_newFloatArray(size_t size);
static float **_new2DFloatArray(size_t rows, size_t cols);

static void RCAC_Scalar(pidProfile_t *pidProfile, const int axisIndex)
{
  const RCAC_input_output_t *inputOutput = &pidProfile->RCAC_input_output[axisIndex];
  const RCAC_hyperparameters_t *hyperParams = &pidProfile->RCAC_hyperparameters[axisIndex];
  RCAC_internal_state_t *state = &pidProfile->RCAC_internal_state[axisIndex];
  const float k = inputOutput->k;
  const uint8_t ltheta = hyperParams->reg_size;
  if (k == 1)
  {
    InitializeWindowBuffers(hyperParams, state);
    InitalizeHBuffer(state, hyperParams);
  }
  UpdateHBuffers(inputOutput, hyperParams, state);
  buildRegressor(state, k, ltheta);
  // updateWindowBuffer(u_in, z_in);
}

static float *_newFloatArray(size_t size)
{
  return (float *)calloc(size, sizeof(float));
}

static float **_new2DFloatArray(size_t rows, size_t cols)
{
  float **array = (float **)malloc(rows * sizeof(float *));
  if (!array)
  {
    return NULL;
  }
  for (size_t idx = 0; idx < rows; idx++)
  {
    array[idx] = (float *)calloc(cols, sizeof(float));
  }
  return array;
}

static void InitalizeHBuffer(RCAC_internal_state_t *internalState, const RCAC_hyperparameters_t *hyperParams)
{
  // Note: u_h is allocated with size Nc-1 while others are size Nc.
  const uint8_t nc = hyperParams->nc;
  // not doing a malloc return type check because if it fails, womp womp system go brbrbrbr!!!!

  free(internalState->u_h);
  internalState->u_h = _newFloatArray(nc - 1); // new float[FLAG.Nc - 1];
  free(internalState->z_h);
  internalState->z_h = _newFloatArray(nc);

  free(internalState->r_h);
  internalState->r_h = _newFloatArray(nc);

  free(internalState->yp_h);
  internalState->yp_h = _newFloatArray(nc);
}

static void InitializeWindowBuffers(const RCAC_hyperparameters_t *hyperParams, RCAC_internal_state_t *state)
{
  // Determine dimensions for the window buffers
  int nf_end = 5;
  int pc = nf_end;
  int pn = pc + hyperParams->FILT_nf + hyperParams->nc;
  const uint8_t ltheta = hyperParams->reg_size;

  // Allocate and initialize PHI_window (dimensions: (ltheta-1) x (pn-1))
  state->PHI_window = _new2DFloatArray(ltheta, pn);

  // Allocate and initialize u_window  and z_window (size: pn-1)
  state->u_window = _newFloatArray(pn);
  state->z_window = _newFloatArray(pn);

  // Allocate and initialize PHI_filt_window (dimensions: (ltheta-1) x (2*ltheta-1))
  state->PHI_filt_window = _new2DFloatArray(ltheta, 2 * ltheta);

  // Allocate and initialize u_filt_window (size: 2*ltheta-1)
  state->u_filt_window = _newFloatArray(2 * ltheta);
}

// UpdateHBuffers: Shifts all history buffers and updates them with new inputs
static void UpdateHBuffers(const RCAC_input_output_t *inputOutput, const RCAC_hyperparameters_t *hyperparams, RCAC_internal_state_t *state)
{
  float u_in = inputOutput->u;
  float z_in = inputOutput->z;
  float yp_in = inputOutput->yp;
  float r_in = inputOutput->r;
  // Update u_h (size: FLAG.Nc - 1)
  int len_u = hyperparams->nc - 1;
  // Shift right: for i from (len_u-1) downto 1, assign u_h[i] = u_h[i-1]
  for (int i = len_u - 1; i > 0; i--)
  {
    state->u_h[i] = state->u_h[i - 1];
  }
  // Set the first element to new control input
  state->u_h[0] = u_in;

  // Update z_h (size: FLAG.Nc)
  int len_z = hyperparams->nc;
  for (int i = len_z - 1; i > 0; i--)
  {
    state->z_h[i] = state->z_h[i - 1];
  }
  state->z_h[0] = z_in;

  // Update r_h (size: FLAG.Nc)
  int len_r = hyperparams->nc;
  for (int i = len_r - 1; i > 0; i--)
  {
    state->r_h[i] = state->r_h[i - 1];
  }
  state->r_h[0] = r_in;

  // Update yp_h (size: FLAG.Nc)
  int len_yp = hyperparams->nc;
  for (int i = len_yp - 1; i > 0; i--)
  {
    state->yp_h[i] = state->yp_h[i - 1];
  }
  // Choose value based on FLAG.RegZ
  if (hyperparams->reg_z == true)
  {
    state->yp_h[0] = z_in;
  }
  else
  {
    state->yp_h[0] = yp_in;
  }

  // Update the integrator variable with the new z_h value
  state->intg = state->intg + state->z_h[0];
}

static void buildRegressor(RCAC_internal_state_t *state, int k, uint8_t ltheta)
{
  if (k == 1)
  {
    state->PHI = _newFloatArray(ltheta);
  }
  state->PHI[0] = state->yp_h[0];
  state->PHI[1] = state->intg;
  state->PHI[2] = state->yp_h[0] - state->yp_h[1];
}

static void updateWindowBuffer(RCAC_internal_state_t *state, const RCAC_hyperparameters_t *hyperparams, float u_in, float z_in)
{
  uint8_t nf_end = 5;
  int32_t nf = hyperparams->FILT_nf;
  int32_t length_window = nf + nf_end;
  uint8_t ltheta = hyperparams->reg_size;

  for (int column = length_window; column > 0; column--)
  {
    state->u_window[column] = state->u_window[column - 1];
  }
  state->u_window[0] = u_in;

  for (int column = length_window; column > 0; column--)
  {
    state->z_window[column] = state->z_window[column - 1];
  }
  state->z_window[0] = z_in;

  for (int row = 0; row < ltheta; row++)
  {
    for (int column = length_window; column > 0; column--)
    {
      state->PHI_window[row][column] = state->PHI_window[row][column - 1];
    }
    state->PHI_window[row][0] = state->PHI[row];
  }
}

static void PrintBuffers(const RCAC_input_output_t *inputOutput,
                  const RCAC_hyperparameters_t *hyperParams,
                  const RCAC_internal_state_t *state)
{
  (void)hyperParams;

  DEBUG_SET(DEBUG_RCAC, 0, lrintf(inputOutput->u));
  DEBUG_SET(DEBUG_RCAC, 1, lrintf(inputOutput->z));
  DEBUG_SET(DEBUG_RCAC, 2, lrintf(inputOutput->yp));
  DEBUG_SET(DEBUG_RCAC, 3, lrintf(inputOutput->r));
  if (state->PHI)
  {
    DEBUG_SET(DEBUG_RCAC, 4, lrintf(state->PHI[0]));
  }
  else
  {
    DEBUG_SET(DEBUG_RCAC, 4, 0);
  }
  DEBUG_SET(DEBUG_RCAC, 5, lrintf(state->u_h[0]));
  DEBUG_SET(DEBUG_RCAC, 6, lrintf(state->z_h[0]));
  DEBUG_SET(DEBUG_RCAC, 7, lrintf(state->r_h[0]));
}

void runRCACController(pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
  UNUSED(currentTimeUs);
  for (int axisIndex = 0; axisIndex < XYZ_AXIS_COUNT; axisIndex++)
  {
    RCAC_input_output_t *inputOutput = &pidProfile->RCAC_input_output[axisIndex];
    inputOutput->k = inputOutput->k + 1;
    RCAC_Scalar(pidProfile, axisIndex);
    PrintBuffers(inputOutput,
                 &pidProfile->RCAC_hyperparameters[axisIndex],
                 &pidProfile->RCAC_internal_state[axisIndex]);
  }
}

void initRCACController(pidProfile_t *pidProfile) {
  // TODO figure out if this is needed
  UNUSED(pidProfile);
}