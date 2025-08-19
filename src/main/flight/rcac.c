#include "rcac.h"

void RCAC_Scalar(const pidProfile_t *pidProfile, const int axisIndex) {
  const RCAC_input_output_t *inputOutput = &pidProfile->RCAC_input_output[axisIndex];
  const RCAC_hyperparameters_t *hyperParams = &pidProfile->RCAC_hyperparameters[axisIndex];
  const RCAC_internal_state_t *state = &pidProfile->RCAC_internal_state[axisIndex];
  const float k = inputOutput->k;
  const uint8_t ltheta = hyperParams->reg_size;
  if (k == 1) {
    InitializeWindowBuffers(state, &hyperParams);
    InitalizeHBuffer(state, &hyperParams);
  }
  UpdateHBuffers(inputOutput, hyperParams, state);
  buildRegressor(state, k, ltheta);
  // updateWindowBuffer(u_in, z_in);
}

float* _newFloatArray(size_t size) {
  return (float*) malloc((size) * sizeof(float));
}

float** _new2DFloatArray(size_t cols, size_t rows) {
  float** array = (float**) malloc(cols * sizeof(float*));
  for (int idx = 0; idx < rows; idx++) {
    array[idx] = (float*) malloc(rows * sizeof(float));
  }
}

void InitalizeHBuffer(RCAC_internal_state_t *internalState, RCAC_hyperparameters_t *hyperParams) {
  // Note: u_h is allocated with size Nc-1 while others are size Nc.
  const uint8_t nc = hyperParams->nc;
  // not doing a malloc return type check because if it fails, womp womp system go brbrbrbr!!!!

  free(internalState->u_h);
  internalState->u_h = _newFloatArray(nc - 1); // new float[FLAG.Nc - 1];

  for (int cols = 0; cols < nc - 1; cols++) {
    internalState->u_h[cols] = 0.0;
  }
  free(internalState->z_h);
  internalState->z_h = _newFloatArray(nc);

  free(internalState->r_h);
  internalState->r_h = _newFloatArray(nc);

  free(internalState->yp_h);
  internalState->yp_h = _newFloatArray(nc);

  for (int cols = 0; cols < nc; cols++) {
    internalState->z_h[cols] = 0.0;
    internalState->r_h[cols] = 0.0;
    internalState->yp_h[cols] = 0.0;
  }
}

void InitializeWindowBuffers(RCAC_hyperparameters_t *hyperParams, RCAC_internal_state_t* state) {
  // Determine dimensions for the window buffers
  int nf_end = 5;
  int pc = nf_end;
  int pn = pc + hyperParams->FILT_nf + hyperParams->nc;
  const uint8_t ltheta = hyperParams->reg_size;

  // Allocate and initialize PHI_window (dimensions: (ltheta-1) x (pn-1))
  state->PHI_window = _new2DFloatArray(ltheta, pn);
  for (int rows = 0; rows < ltheta - 1; rows++) {
    for (int cols = 0; cols < pn - 1; cols++) {
      state->PHI_window[rows][cols] = 0.0;
    }
  }

  // Allocate and initialize u_window  and z_window (size: pn-1)
  state->u_window = _newFloatArray(pn);
  state->z_window = _newFloatArray(pn);
  for (int cols = 0; cols < pn - 1; cols++) {
    state->u_window[cols] = 0.0;
    state->z_window[cols] = 0.0;
  }

  // Allocate and initialize PHI_filt_window (dimensions: (ltheta-1) x (2*ltheta-1))
  state->PHI_filt_window = _new2DFloatArray(ltheta, 2*ltheta);
  for (int rows = 0; rows < ltheta - 1; rows++) {
    for (int cols = 0; cols < 2 * ltheta - 1; cols++) {
      state->PHI_filt_window[rows][cols] = 0.0;
    }
  }

  // Allocate and initialize u_filt_window (size: 2*ltheta-1)
  state->u_filt_window = _newFloatArray(2 * ltheta);
  for (int cols = 0; cols < 2 * ltheta - 1; cols++) {
    state->u_filt_window[cols] = 0.0;
  }
}

// UpdateHBuffers: Shifts all history buffers and updates them with new inputs
void UpdateHBuffers(RCAC_input_output_t* inputOutput, RCAC_hyperparameters_t* hyperparams, RCAC_internal_state_t* state) {
  float u_in = inputOutput->u;
  float z_in = inputOutput->z;
  float yp_in = inputOutput->yp;
  float r_in = inputOutput->r;
  // Update u_h (size: FLAG.Nc - 1)
  int len_u = hyperparams->nc - 1;
  // Shift right: for i from (len_u-1) downto 1, assign u_h[i] = u_h[i-1]
  for (int i = len_u - 1; i > 0; i--) {
    state->u_h[i] = state->u_h[i - 1];
  }
  // Set the first element to new control input
  state->u_h[0] = u_in;

  // Update z_h (size: FLAG.Nc)
  int len_z = hyperparams->nc;
  for (int i = len_z - 1; i > 0; i--) {
    state->z_h[i] = state->z_h[i - 1];
  }
  state->z_h[0] = z_in;

  // Update r_h (size: FLAG.Nc)
  int len_r = hyperparams->nc;
  for (int i = len_r - 1; i > 0; i--) {
    state->r_h[i] = state->r_h[i - 1];
  }
  state->r_h[0] = r_in;

  // Update yp_h (size: FLAG.Nc)
  int len_yp = hyperparams->nc;
  for (int i = len_yp - 1; i > 0; i--) {
    state->yp_h[i] = state->yp_h[i - 1];
  }
  // Choose value based on FLAG.RegZ
  if (hyperparams->reg_z == true) {
    state->yp_h[0] = z_in;
  } else {
    state->yp_h[0] = yp_in;
  }

  // Update the integrator variable with the new z_h value
  state->intg = state->intg + state->z_h[0];
}

void buildRegressor(RCAC_internal_state_t* state, int k, uint8_t ltheta) {
  if (k == 1) {
    state->PHI = _newFloatArray(ltheta);
  }
  state->PHI[0] = state->yp_h[0];
  state->PHI[1] = state->intg;
  state->PHI[2] = state->yp_h[0] - state->yp_h[1];
}

void updateWindowBuffer(RCAC_internal_state_t* state, RCAC_hyperparameters_t* hyperparams, float u_in, float z_in) {
  uint8_t nf_end = 5;
  int32_t nf = hyperparams->FILT_nf;
  int32_t length_window = nf + nf_end;
  uint8_t ltheta = hyperparams->reg_size;

  for (int column = length_window; column > 0; column--) {
    state->u_window[column] = state->u_window[column - 1];
  }
  state->u_window[0] = u_in;

  for (int column = length_window; column > 0; column--) {
    state->z_window[column] = state->z_window[column - 1];
  }
  state->z_window[0] = z_in;


  for (int row = 0; row < ltheta; row++) {
    for (int column = length_window; column > 0; column--) {
      state->PHI_window[row][column] = state->PHI_window[row][column - 1];
    }
    state->PHI_window[row][0] = state->PHI[row];
  }
}


void PrintBuffers() {
  // Print H Buffers
  Serial.println("=== H Buffers ===");

  Serial.print("u_h: ");
  for (int i = 0; i < FLAG.Nc - 1; i++) {
    Serial.print(u_h[i]);
    if (i < FLAG.Nc - 2)
      Serial.print(", ");
  }
  Serial.println();

  Serial.print("z_h: ");
  for (int i = 0; i < FLAG.Nc; i++) {
    Serial.print(z_h[i]);
    if (i < FLAG.Nc - 1)
      Serial.print(", ");
  }
  Serial.println();

  Serial.print("r_h: ");
  for (int i = 0; i < FLAG.Nc; i++) {
    Serial.print(r_h[i]);
    if (i < FLAG.Nc - 1)
      Serial.print(", ");
  }
  Serial.println();

  Serial.print("yp_h: ");
  for (int i = 0; i < FLAG.Nc; i++) {
    Serial.print(yp_h[i]);
    if (i < FLAG.Nc - 1)
      Serial.print(", ");
  }
  Serial.println();

  // Recalculate window dimensions for printing purposes
  static const int nf_end = 5;
  static const int pc = nf_end;
  static const int pn = pc + FILT.Nf + FLAG.Nc;

  // Print Window Buffers
  Serial.println("=== Window Buffers ===");
  Serial.println("PHI_window:");
  for (int i = 0; i < ltheta - 1; i++) {
    for (int j = 0; j < pn - 1; j++) {
      Serial.print(PHI_window[i][j]);
      if (j < pn - 2)
        Serial.print(", ");
    }
    Serial.println();
  }

  Serial.print("u_window: ");
  for (int i = 0; i < pn - 1; i++) {
    Serial.print(u_window[i]);
    if (i < pn - 2)
      Serial.print(", ");
  }
  Serial.println();

  Serial.print("z_window: ");
  for (int i = 0; i < pc - 1; i++) {
    Serial.print(z_window[i]);
    if (i < pc - 2)
      Serial.print(", ");
  }
  Serial.println();

  // Print Filt Window Buffers
  Serial.println("=== Filt Window Buffers ===");
  Serial.println("PHI_filt_window:");
  for (int i = 0; i < ltheta - 1; i++) {
    for (int j = 0; j < 2 * ltheta - 1; j++) {
      Serial.print(PHI_filt_window[i][j]);
      if (j < 2 * ltheta - 2)
        Serial.print(", ");
    }
    Serial.println();
  }

  Serial.print("u_filt_window: ");
  for (int i = 0; i < 2 * ltheta - 1; i++) {
    Serial.print(u_filt_window[i]);
    if (i < 2 * ltheta - 2)
      Serial.print(", ");
  }
  Serial.println();

  // // Print the constant matrix P_k
  // Serial.println("=== Constant Matrix P_k ===");
  // for (int i = 0; i < 3; i++) {
  //   for (int j = 0; j < 3; j++) {
  //     Serial.print(P_k[i][j], 6);
  //     if (j < 2)
  //       Serial.print(", ");
  //   }
  //   Serial.println();
  // }

  // Print the regressor vector PHI
  Serial.println("=== Regressor PHI ===");
  if (PHI != NULL) {
    for (int i = 0; i < ltheta; i++) {
      Serial.print(PHI[i]);
      if (i < ltheta - 1) {
        Serial.print(", ");
      }
    }
    Serial.println();
  } else {
    Serial.println("PHI is not allocated.");
  }
}

void initRCACController(const pidProfile_t *pidProfile) {
  for (int axisIndex = 0; axisIndex < XYZ_AXIS_COUNT; axisIndex++) {
    RCAC_Scalar(pidProfile, axisIndex);
    PrintBuffers();
  }
}

void loop() {
  // Your main loop logic
  u = u + 0.1; // u is your control output of rcac, this is the past rcac u output. generally this will be your motor speed
  z = z + 0.11; // this is you 'performance vector' as known in the literature or just error. this is what you want to go to zero when good perfoamcne is met. ie, z = commanded pitch rate - measured pitch rate
  yp = yp + 0.12; // this is your measurement, ie pitch rate, yaw rate, or roll rate. 
  r = r + 0.13; // this is your command, appologies for the terrible notation, ie commanded pitch rate
  k = k + 1; // this is your itteration, this is only used for startup and or initalization when k = 1.
  RCAC_Scalar(k, u, z, yp, r); // this is single input single output RCAC running 
  PrintBuffers();
  delay(1000);
}
