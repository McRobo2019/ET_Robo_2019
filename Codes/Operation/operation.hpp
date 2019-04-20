
/******************************************************************************
 *****************************************************************************/

#ifndef EV3_UNIT_BALANCINGWALKER_H_
#define EV3_UNIT_BALANCINGWALKER_H_

#include "util.hpp"
#include "parameter.h"
#include "GyroSensor.h"
#include "Motor.h"


using namespace std;

#define PWM_ABS_MAX       60 /* 完全停止用モータ制御PWM絶対最大値 */

class Operation {
public:
  static const int LOW;
  static const int NORMAL;
  static const int HIGH;

  Operation(const ev3api::GyroSensor& gyroSensor,
	   ev3api::Motor& leftWheel,
	   ev3api::Motor& rightWheel,
	   ev3api::Motor& arm_motor,	  
	   ev3api::Motor& tail_motor);

  void init();
  void setCommand(float velocity, int forward, float yawratecmd, float yawrate);
  void set_robo_mode_launch();

  void run();

private:
  const ev3api::GyroSensor& mGyroSensor;
  ev3api::Motor& mLeftWheel;
  ev3api::Motor& mRightWheel;
  ev3api::Motor& mArm_Motor;
  ev3api::Motor& mTail_Motor;

  PID *gForward  = new PID();

  enum Robo_Mode{
    SET,
    READY,
    LAUNCH,
    RUN,
    ROBO_DEBUG
  };

  Robo_Mode  ROBO_MODE;

  float mVelocity        = 0.0;
  int   mForward;
  float mTurn;
  float mYawratecmd;//目標Yawrate
  float mYawrate;

  int   mTarget_forward  = 0;
  float mCurved_forward  = 0;
  int   mCurrent_forward = 0;

  int   mRef_velocity  = 0;


  bool  mForward_curve_mode;
   
  int right_wheel_enc        = 0;
  int left_wheel_enc         = 0;


  float YawrateController(float yawrate, float yawrate_cmd);
  float yaw_ctl_dt = 0.004;

  float turn_tmp   = 0.0;
  float r_yaw_rate = 0.0;
  float r_yaw_rate_ud = 0.0;
  float I_gain1 = 1.0+1.0/0.8;
  float I_gain2 = yaw_ctl_dt/0.06/0.8;
  float I_gain3 = 1.0-yaw_ctl_dt/0.06;

  float F_controller(float r_yaw_rate);
  float F_in = 0.0;
  float F_out = 0.0;
  //    float F_gain = 1/0.062;
  float F_gain = 1/0.062; //0818 tada
    
  float E_controller(float r_yaw_rate);
  float E_in;
  float E_in_d;
  float E_in_dd;
  float E_in_ddd;
  float E_in_dddd;
  float E_in_ddddd = 0.0;
  float E_in_dddddd = 0.0;

  float E_out = 0.0;
  float E_gain1 = yaw_ctl_dt/0.1;
  float E_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
  float E_ud1 = 0.0;

  float C_controller(float E_out, float yawrate, float S_out);
  float C_in = 0.0;
  float C_out = 0.0;
  //float C_gain = yaw_ctl_dt*10.0;
  float C_gain = yaw_ctl_dt*50.0;
  float C_ud1 = 0.0;

  float S_controller(float C_out);
  float S_gain1 = 1.0;
  float S_in;
  float S_in_d;
  float S_in_dd;
  float S_in_ddd;
  float S_in_dddd;
  float S_in_ddddd = 0.0;
  float S_in_dddddd = 0.0;
  float S_out = 0.0;
  float Pn_gain1 = yaw_ctl_dt/0.1;
  float Pn_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
  float Pn_ud1 = 0.0;
  float Pd_gain1 = yaw_ctl_dt/0.1;
  float Pd_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
  float Pd_ud1 = 0.0;

  void PWM_Gen(int mForward, float mTurn);
  int left_motor_pwm;
  int right_motor_pwm;


};

#endif  // EV3_UNIT_BALANCINGWALKER_H_
