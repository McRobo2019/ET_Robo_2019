
/******************************************************************************
 *****************************************************************************/

#ifndef EV3_UNIT_BALANCINGWALKER_H_
#define EV3_UNIT_BALANCINGWALKER_H_

#include "util.hpp"
#include "parameter.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "yawrate_ctl.hpp"


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
  void setCommand(float velocity, int forward, float target_yaw_rate, float yawrate);
  void set_robo_mode_launch();

  void run();

  void arm_reset();
  void arm_stand_up(); //tail for gyro reset and color sensor calibration

  int left_motor_pwm;
  int right_motor_pwm;


private:
  const ev3api::GyroSensor& mGyroSensor;
  ev3api::Motor& mLeftWheel;
  ev3api::Motor& mRightWheel;
  ev3api::Motor& mArm_Motor;
  ev3api::Motor& mTail_Motor;

  PID *gForward  = new PID();
  Yawrate_Ctl *gYawrate_Ctl = new Yawrate_Ctl();

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
  float mTarget_Yaw_Rate;//目標Yawrate
  float mYawrate;

  int   mTarget_forward  = 0;
  float mCurved_forward  = 0;
  int   mCurrent_forward = 0;

  int   mRef_velocity  = 0;


  bool  mForward_curve_mode;
   
  int right_wheel_enc        = 0;
  int left_wheel_enc         = 0;


  void PWM_Gen(int mForward, float mTurn);


};

#endif  // EV3_UNIT_BALANCINGWALKER_H_
