/******************************************************************************
 *  Author: Kaoru Ota
 *****************************************************************************/

#include "operation.hpp"
#include "Clock.h"

using ev3api::Clock;

Clock*       robo_Clock;


// 定数宣言
const int Operation::LOW    = 30;    // 低速
const int Operation::NORMAL = 50;    // 通常
const int Operation::HIGH   = 70;    // 高速

/**
 * コンストラクタ
 * @param gyroSensor ジャイロセンサ
 * @param leftWheel  左モータ
 * @param rightWheel 右モータ
 * @param balancer   バランサ
 */
Operation::Operation(const
		     ev3api::GyroSensor& gyroSensor,
		     ev3api::Motor& leftWheel,
		     ev3api::Motor& rightWheel,
		     ev3api::Motor& arm_motor,     
		     ev3api::Motor& tail_motor
		     )
    : mGyroSensor(gyroSensor),
      mLeftWheel(leftWheel),
      mRightWheel(rightWheel),
      mArm_Motor(arm_motor),
      mTail_Motor(tail_motor),
      mForward(LOW),
      mTurn(LOW)
 {
}


void Operation::init() {

  robo_Clock       = new Clock();        

  mLeftWheel.reset();
  mRightWheel.reset();

  gLeft_Motor_ctlModelClass->initialize();
  gRight_Motor_ctlModelClass->initialize();
  //  gForward->init_pid(0.05,0.01,0.001,dT_4ms);
  gForward->init_pid(0.05,0.01,0.001,dT_10ms);

  ROBO_MODE  = SET;

  checked_target_velocity = 0;
  checked_target_omega = 0;

  pre_target_velocity = 0;
  pre_target_omega = 0;

  monitor_error = false;



}

/**
 * PWM値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */

void Operation::setCommand(float velocity, float left_wheel_velocity, float right_wheel_velocity, int forward, float target_yaw_rate, float yawrate, int target_velocity, float target_omega){

  mVelocity             = velocity;
  mLeft_Wheel_Velocity  = left_wheel_velocity;
  mRight_Wheel_Velocity = right_wheel_velocity;

  mForward            = forward;
  mTarget_Yaw_Rate    = target_yaw_rate;
  mYawrate            = yawrate;
  
  mTarget_Velocity    = target_velocity;
  mTarget_Omega       = target_omega;
}

void Operation::monitoring_cmd(){

  int diff_velocity;
  float diff_omega;

  diff_velocity = mTarget_Velocity - pre_target_velocity;
  if(diff_velocity < 0 ){
    diff_velocity = -1 * diff_velocity;
  }

  if(diff_velocity > 1000){
    checked_target_velocity = pre_target_velocity;
    monitor_error = true;
  }else{
    checked_target_velocity = mTarget_Velocity;
    pre_target_velocity     = mTarget_Velocity;
    monitor_error = false;
  }

  diff_omega = mTarget_Omega - pre_target_omega;
  if(diff_omega < 0 ){
    diff_omega = -1.0 * diff_omega;
  }

  if(diff_omega > 100){
    checked_target_omega = pre_target_omega;
    monitor_error = true;
  }else{
    checked_target_omega = mTarget_Omega;
    pre_target_omega     = mTarget_Omega;
    monitor_error = false;
  }
}



void Operation::set_robo_mode_launch(){
  ROBO_MODE = LAUNCH;
}

void Operation::run() {
  //    int     battery       = ev3_battery_voltage_mV();
  float vl;
  float vr;
  float l_pwm;
  float r_pwm;

  monitoring_cmd();

  switch(ROBO_MODE){

  case SET:
    
    break;

  case READY:

    break;


  case LAUNCH:
    ROBO_MODE = RUN;

    break;
    
  case RUN:
    right_wheel_enc = mRightWheel.getCount();
    left_wheel_enc  = mLeftWheel.getCount();             // 左モータ回転角度

    //190620 ota
    //      vl = (float)mTarget_Velocity;
    vl = (float)checked_target_velocity;
    vr = vl;
    vl = vl - (checked_target_omega * HALF_TREAD);
    vr = vr + (checked_target_omega * HALF_TREAD);
    
    /*
      vl = vl + 0.5;
      vr = vr + 0.5;
    */
    gLeft_Motor_ctlModelClass->setIn1(vl);
    gLeft_Motor_ctlModelClass->setIn2(mLeft_Wheel_Velocity);
    gLeft_Motor_ctlModelClass->step();
    l_pwm = gLeft_Motor_ctlModelClass->getOut1();
    left_motor_pwm = (int)l_pwm;

    
    gRight_Motor_ctlModelClass->setIn1(vr);
    gRight_Motor_ctlModelClass->setIn2(mRight_Wheel_Velocity);
    gRight_Motor_ctlModelClass->step();
    r_pwm = gRight_Motor_ctlModelClass->getOut1();
    right_motor_pwm = (int)r_pwm;


    /*
      mTurn = gYawrate_Ctl->YawrateController(mYawrate, mTarget_Yaw_Rate);
      PWM_Gen(mForward, mTurn);
    */
      
    mLeftWheel.setPWM(left_motor_pwm);
    mRightWheel.setPWM(right_motor_pwm);

    break;

  case ROBO_DEBUG:
    break;
  }

}


//2017.07.28 k-ota copy from 3-apex
//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************

/*
void Operation::tail_control(signed int angle)
{
  tail_motor_pwm = gTail_pwm->calc_pid(angle, mTail_Motor.getCount());
  tail_motor_pwm = tail_motor_pwm*0.1;

  if (tail_motor_pwm > PWM_ABS_MAX)
    {
      tail_motor_pwm = PWM_ABS_MAX;
    }
  else if (tail_motor_pwm < -PWM_ABS_MAX)
    {
      tail_motor_pwm = -PWM_ABS_MAX;
    }

  if (tail_motor_pwm == 0)
    {
      //17.07.28 kota modify//        ev3_motor_stop(tail_motor, true);
      mTail_Motor.stop();
    }
  else
    {
      //17.07.28 kota modify//        ev3_motor_set_power(tail_motor, (signed char)pwm);
      mTail_Motor.setPWM((signed int)tail_motor_pwm);
    }
}
*/


/*
void Operation::tail_reset(){
  int32_t angle    = 0;
  int32_t angle_1d = 0;

  mTail_Motor.setPWM(-10);
  angle = 0;
  angle_1d = 1;

  while(1){
    if(angle == angle_1d){
      mTail_Motor.stop();
      mTail_Motor.reset();
      break;
    }
    else{
      angle_1d = angle;
      tslp_tsk(1000);
      angle = mTail_Motor.getCount();
    }
  }
  mTail_Motor.stop();
  mTail_Motor.reset();
}
*/

/*
void Operation::tail_stand_up(){
    while(1){
      if(mTail_Motor.getCount() == TAIL_ANGLE_STAND_UP){
	mTail_Motor.stop();
	break;
      }
      else{
	mTail_Motor.setPWM(5);
      }
    }
    mTail_Motor.stop();
} //tail for gyro reset and color sensor calibration
*/


void Operation::PWM_Gen(int mForward, float mTurn){
  left_motor_pwm  = 0.5*mForward + 1.0*mTurn;
  right_motor_pwm = 0.5*mForward - 1.0*mTurn;
}

