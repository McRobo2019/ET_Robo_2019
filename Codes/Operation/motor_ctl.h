//
// File: motor_ctl.h
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
#ifndef RTW_HEADER_motor_ctl_h_
#define RTW_HEADER_motor_ctl_h_
#ifndef motor_ctl_COMMON_INCLUDES_
# define motor_ctl_COMMON_INCLUDES_
#include "rtwtypes.h"
//#include "rtw_continuous.h"
//#include "rtw_solver.h"
#endif                                 // motor_ctl_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM RT_MODEL;

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  real_T UnitDelay1_DSTATE;            // '<S2>/Unit Delay1'
} DW;

// External inputs (root inport signals with default storage)
typedef struct {
  real_T In1;                          // '<Root>/In1'
  real_T In2;                          // '<Root>/In2'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T Out1;                         // '<Root>/Out1'
} ExtY;

// Real-time Model Data Structure
struct tag_RTM {
  const char_T * volatile errorStatus;
};

// Class declaration for model motor_ctl
class motor_ctlModelClass {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  motor_ctlModelClass();

  // Destructor
  ~motor_ctlModelClass();

  // Root inport: '<Root>/In1' set method
  void setIn1(real_T localArgInput);

  // Root inport: '<Root>/In2' set method
  void setIn2(real_T localArgInput);

  // Root outport: '<Root>/Out1' get method
  real_T getOut1() const;

  // Real-Time Model get method
  RT_MODEL * getRTM();

  // protected data and function members
 protected:
  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;

  // Real-Time Model
  RT_MODEL rtM;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('motor_pi/motor_ctl')    - opens subsystem motor_pi/motor_ctl
//  hilite_system('motor_pi/motor_ctl/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'motor_pi'
//  '<S1>'   : 'motor_pi/motor_ctl'
//  '<S2>'   : 'motor_pi/motor_ctl/pi_ctl'

#endif                                 // RTW_HEADER_motor_ctl_h_

//
// File trailer for generated code.
//
// [EOF]
//
