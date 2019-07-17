//
// File: LPF_CF5HZ.cpp
//
// Code generated for Simulink model 'LPF_CF5HZ'.
//
// Model version                  : 1.1
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Tue Jul  9 16:46:32 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 9
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "LPF_CF5HZ.h"

// Model step function
void LPF_CF5HZModelClass::step()
{
  int32_T j;
  real_T rtb_Filter1;

  // Outputs for Atomic SubSystem: '<Root>/LPF_CF5HZ'
  // DiscreteFir: '<S1>/Filter1' incorporates:
  //   Inport: '<Root>/In1'

  // Consume delay line and beginning of input samples
  rtb_Filter1 = rtU.In1 * rtConstP.Filter1_Coefficients[0];
  for (j = 0; j < 100; j++) {
    rtb_Filter1 += rtConstP.Filter1_Coefficients[1 + j] * rtDW.Filter1_states[j];
  }

  // Update delay line for next frame
  for (j = 98; j >= 0; j--) {
    rtDW.Filter1_states[1 + j] = rtDW.Filter1_states[j];
  }

  rtDW.Filter1_states[0] = rtU.In1;

  // End of DiscreteFir: '<S1>/Filter1'
  // End of Outputs for SubSystem: '<Root>/LPF_CF5HZ'

  // Outport: '<Root>/Out1'
  rtY.Out1 = rtb_Filter1;
}

// Model initialize function
void LPF_CF5HZModelClass::initialize()
{
  // (no initialization code required)
}

// Constructor
LPF_CF5HZModelClass::LPF_CF5HZModelClass()
{
  // Currently there is no constructor body generated.
}

// Destructor
LPF_CF5HZModelClass::~LPF_CF5HZModelClass()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//
