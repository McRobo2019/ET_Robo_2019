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


  gForward->init_pid(0.05,0.01,0.001,dT_4ms);

  ROBO_MODE  = SET;
}

/**
 * PWM値を設定する
 * @param forward 前進値
 * @param turn    旋回値
 */
void Operation::setCommand(float velocity, int forward, float yawratecmd, float yawrate) {

  mVelocity           = velocity;
  mForward            = forward;
  mYawratecmd         = yawratecmd;
  mYawrate            = yawrate;
}

void Operation::set_robo_mode_launch(){
  ROBO_MODE = LAUNCH;
}

void Operation::run() {
  //    int     battery       = ev3_battery_voltage_mV();

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

      mTurn = gYawrate_Ctl->YawrateController(mYawrate, mYawratecmd);
      PWM_Gen(mForward, mTurn);
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

