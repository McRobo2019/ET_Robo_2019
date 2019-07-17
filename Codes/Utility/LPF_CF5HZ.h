//
// File: LPF_CF5HZ.h
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
#ifndef RTW_HEADER_LPF_CF5HZ_h_
#define RTW_HEADER_LPF_CF5HZ_h_
#ifndef LPF_CF5HZ_COMMON_INCLUDES_
# define LPF_CF5HZ_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // LPF_CF5HZ_COMMON_INCLUDES_

// Macros for accessing real-time model data structure

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  real_T Filter1_states[100];          // '<S1>/Filter1'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Expression: [0.00456087975259436649 -0.00181663348337506698 -0.00182639167312229913 -0.00199240452484161909 -0.0022381839856466107 -0.00249538444821011682 -0.00270291804299660487 -0.00280510532316929299 -0.00275500046501173232 -0.00251725175002385972 -0.00207315439026600117 -0.00141750753523047271 -0.000561391998989817904 0.000458416599144693124 0.00159556605357077854 0.00277983075809552175 0.00393173632424987166 0.00496174280142075777 0.00577693930094417153 0.00628786667645815384 0.00641458332502943453 0.00609591379136947917 0.00529250440078157022 0.00399595645083018237 0.00223159266608123543 5.8977961717700188e-05 -0.00242347014642375382 -0.00508538676123084909 -0.00776561106152528786 -0.0102807076974250691 -0.0124333183047034557 -0.0140219846326852648 -0.0148549422334195956 -0.0147567357170871622 -0.013587180750866415 -0.0112447114986979572 -0.00767891398850178725 -0.00289744108858680733 0.00303400468245583086 0.00998796737258814821 0.0177796041025792204 0.0261724948139632797 0.034889006274162998 0.0436249336662691278 0.0520490400094173813 0.0598553759775868402 0.0667176897962276083 0.0723712769066276046 0.0765895410024716866 0.0791945864731981031 0.0800755787018147397 0.0791945864731981031 0.0765895410024716866 0.0723712769066276046 0.0667176897962276083 0.0598553759775868402 0.0520490400094173813 0.0436249336662691278 0.034889006274162998 0.0261724948139632797 0.0177796041025792204 0.00998796737258814821 0.00303400468245583086 -0.00289744108858680733 -0.00767891398850178725 -0.0112447114986979572 -0.013587180750866415 -0.0147567357170871622 -0.0148549422334195956 -0.0140219846326852648 -0.0124333183047034557 -0.0102807076974250691 -0.00776561106152528786 -0.00508538676123084909 -0.00242347014642375382 5.8977961717700188e-05 0.00223159266608123543 0.00399595645083018237 0.00529250440078157022 0.00609591379136947917 0.00641458332502943453 0.00628786667645815384 0.00577693930094417153 0.00496174280142075777 0.00393173632424987166 0.00277983075809552175 0.00159556605357077854 0.000458416599144693124 -0.000561391998989817904 -0.00141750753523047271 -0.00207315439026600117 -0.00251725175002385972 -0.00275500046501173232 -0.00280510532316929299 -0.00270291804299660487 -0.00249538444821011682 -0.0022381839856466107 -0.00199240452484161909 -0.00182639167312229913 -0.00181663348337506698 0.00456087975259436649]
  //  Referenced by: '<S1>/Filter1'

  real_T Filter1_Coefficients[101];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  real_T In1;                          // '<Root>/In1'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T Out1;                         // '<Root>/Out1'
} ExtY;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Class declaration for model LPF_CF5HZ
class LPF_CF5HZModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  LPF_CF5HZModelClass();

  // Destructor
  ~LPF_CF5HZModelClass();

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;
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
//  hilite_system('LPF/LPF_CF5HZ')    - opens subsystem LPF/LPF_CF5HZ
//  hilite_system('LPF/LPF_CF5HZ/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'LPF'
//  '<S1>'   : 'LPF/LPF_CF5HZ'

#endif                                 // RTW_HEADER_LPF_CF5HZ_h_

//
// File trailer for generated code.
//
// [EOF]
//
