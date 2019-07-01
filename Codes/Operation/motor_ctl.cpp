//
// File: motor_ctl.cpp
//
// Code generated for Simulink model 'motor_ctl'.
//
// Model version                  : 1.10
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri Jun 28 09:16:03 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 9
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "motor_ctl.h"

// Model step function
void motor_ctlModelClass::step()
{
  real_T rtb_Sum3;
  real_T rtb_Sum1;

  // Outputs for Atomic SubSystem: '<Root>/motor_ctl'
  // Sum: '<S1>/Sum' incorporates:
  //   Inport: '<Root>/In1'
  //   Inport: '<Root>/In2'

  rtb_Sum3 = rtU.In1 - rtU.In2;

  // Sum: '<S2>/Sum1' incorporates:
  //   Gain: '<S2>/Ki'
  //   Gain: '<S2>/Ts'
  //   UnitDelay: '<S2>/Unit Delay1'

  rtb_Sum1 = 0.01 * rtb_Sum3 * 0.47782874617737003 + rtDW.UnitDelay1_DSTATE;

  // Update for UnitDelay: '<S2>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE = rtb_Sum1;

  // Sum: '<S2>/Sum3' incorporates:
  //   Gain: '<S2>/Kp'

  rtb_Sum3 = 0.0382262996941896 * rtb_Sum3 + rtb_Sum1;

  // Saturate: '<S1>/Saturation'
  if (rtb_Sum3 > 100.0) {
    // Outport: '<Root>/Out1'
    rtY.Out1 = 100.0;
  } else if (rtb_Sum3 < 0.0) {
    // Outport: '<Root>/Out1'
    rtY.Out1 = 0.0;
  } else {
    // Outport: '<Root>/Out1'
    rtY.Out1 = rtb_Sum3;
  }

  // End of Saturate: '<S1>/Saturation'
  // End of Outputs for SubSystem: '<Root>/motor_ctl'
}

// Model initialize function
void motor_ctlModelClass::initialize()
{
  // (no initialization code required)
}

// Constructor
motor_ctlModelClass::motor_ctlModelClass()
{
  // Currently there is no constructor body generated.
}

// Destructor
motor_ctlModelClass::~motor_ctlModelClass()
{
  // Currently there is no destructor body generated.
}

// Root-level input access methods

// Root inport: '<Root>/In1' set method
void motor_ctlModelClass::setIn1(real_T localArgInput)
{
  rtU.In1 = localArgInput;
}

// Root inport: '<Root>/In2' set method
void motor_ctlModelClass::setIn2(real_T localArgInput)
{
  rtU.In2 = localArgInput;
}

// Root-level output access methods

// Root outport: '<Root>/Out1' get method
real_T motor_ctlModelClass::getOut1() const
{
  return rtY.Out1;
}

// Real-Time Model get method
RT_MODEL * motor_ctlModelClass::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
