
/******************************************************************************
 *****************************************************************************/

#ifndef EV3_UNIT_BALANCINGWALKER_H_
#define EV3_UNIT_BALANCINGWALKER_H_

#include "util.hpp"
#include "parameter.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "yawrate_ctl.hpp"
#include "motor_ctl.h"


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
  //  void setCommand(float velocity, int forward, float target_yaw_rate, float yawrate, int target_velocity, float target_omega);
  void setCommand(float velocity, float left_wheel_velocity, float right_wheel_velocity, int forward, float target_yaw_rate, float yawrate, int target_velocity, float target_omega);
  void set_robo_mode_launch();

  void arm_reset();
  void arm_line_trace();
  void arm_control(signed int angle);
  void run();

  //190620 ota
  int target_right_velocity; //target right wheel velocity 
  int target_left_velocity;  //target left wheel velocity

  int left_motor_pwm;
  int right_motor_pwm;
 

private:
  const ev3api::GyroSensor& mGyroSensor;
  ev3api::Motor& mLeftWheel;
  ev3api::Motor& mRightWheel;
  ev3api::Motor& mArm_Motor;
  ev3api::Motor& mTail_Motor;

  PID *gArm_pwm = new PID();

  PID *gForward  = new PID();
  Yawrate_Ctl *gYawrate_Ctl = new Yawrate_Ctl();
  motor_ctlModelClass *gLeft_Motor_ctlModelClass = new motor_ctlModelClass;
  motor_ctlModelClass *gRight_Motor_ctlModelClass = new motor_ctlModelClass;


  enum Robo_Mode{
    SET,
    READY,
    LAUNCH,
    RUN,
    ROBO_DEBUG
  };

  Robo_Mode  ROBO_MODE;

  float mVelocity        = 0.0;
  float mLeft_Wheel_Velocity  = 0.0;
  float mRight_Wheel_Velocity = 0.0;

  int   mForward;
  float mTurn;
  float mTarget_Yaw_Rate;//目標Yawrate
  float mYawrate;
  //190620
  int   mTarget_Velocity;
  float mTarget_Omega;
  //
  int   mTarget_forward  = 0;
  float mCurved_forward  = 0;
  int   mCurrent_forward = 0;

  int   mRef_velocity  = 0;


  bool  mForward_curve_mode;
   
  float arm_motor_pwm;

  int right_wheel_enc        = 0;
  int left_wheel_enc         = 0;

  int   checked_target_velocity = 0;
  float checked_target_omega = 0;

  int   pre_target_velocity = 0;
  float pre_target_omega = 0;



  bool  monitor_error = false;

  void monitoring_cmd();

  void PWM_Gen(int mForward, float mTurn);


};

#endif  // EV3_UNIT_BALANCINGWALKER_H_
