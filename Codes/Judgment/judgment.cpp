/******************************************************************************
 *  Author: Koaru Ota

--- basic structure ---

*****************************************************************************/

#include <stdlib.h>
#include "ev3api.h"
#include "judgment.hpp"


Judgment::Judgment() {

}

void Judgment::init() {
  ZONE               = START_ZONE;
  DRIVE_MODE         = LINE_TRACE;
  ON_LINE_MODE       = ON_THE_LEFT_EDGE;
  PRE_ON_LINE_MODE   = ON_THE_LEFT_EDGE;
  

  on_line            = true;
  left_line          = false;
  right_line         = false;
  lost_line          = false;
  line_to_map        = false; //181108


  line_trace_mode    = true;

  gAve_line_val->init();
  gAve_yaw_angle_500->init(); //20181108

  //  gMotion_Ctl->init();

  mMax_Forward = 10;

  det_navi_log = 0;

  re_start     = false; //181112

}


void Judgment::set_drive_mode_LT(){
  DRIVE_MODE = LINE_TRACE;
  line_trace_mode = true;
}

void Judgment::set_drive_mode_TK(){
  DRIVE_MODE = TRACK;
  line_trace_mode = false;
}

void Judgment::set_drive_mode_DB(){
  DRIVE_MODE = DEBUG;
  line_trace_mode = false;
}



void Judgment::run() {

  ave_line_val = gAve_line_val->average_500(mLinevalue);
  ave_yaw_angle_500 = gAve_yaw_angle_500->average_500(mYawangle);

  //  det_on_line(); //it has not been completed yet, so it will not work well 20190414 ota

  det_navigation();

  /* 0525
  gMotion_Ctl->SetCurrentData(mLinevalue,
			      mGreen_flag,
			      mXvalue,
			      mYvalue,
			      mPre_50mm_x,
			      mPre_50mm_y,
			      mOdo,
			      mVelocity,
			      mYawrate,
			      mYawangle,
			      mTail_angle,
			      mRobo_stop,
			      mRobo_forward,
			      mRobo_back,
			      mRobo_turn_left,
			      mRobo_turn_right,
			      mSonar_dis,
			      mMax_Forward,
			      mRef_Yawrate,
			      mMax_Yawrate,
			      mMin_Yawrate
			      );

  gMotion_Ctl->run(mXvalue,mYvalue,mYawangle);


  GetCalcResult(gMotion_Ctl->forward,
		gMotion_Ctl->yawratecmd,
		gMotion_Ctl->ref_tail_angle,
		gMotion_Ctl->tail_stand_mode);

  */

}

/****************************************************************************************/
//2018 04 21 Kaoru Ota
//    //https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo
/****************************************************************************************/
void Judgment::det_navigation() {

  /*  float yaw_time;

  static float ref_odo;
  static float dif_odo;

  static int   ref_forward;
  static float acl_forward;
  */

  if(DRIVE_MODE == LINE_TRACE){
    line_trace_mode = true;

    forward    = 30;    

    mRef_Yawrate = 0.0;
    mMax_Yawrate = RAD_45_DEG;
    mMin_Yawrate = MINUS_RAD_45_DEG;

    yawratecmd = gLine_Trace->line_trace_yaw_rate(mLinevalue, mRef_Yawrate, mMax_Yawrate, mMin_Yawrate);

  }else if(DRIVE_MODE == TRACK){
    line_trace_mode    = false;

  }else if(DRIVE_MODE == DEBUG){
    line_trace_mode    = false;
  }
}

/****************************************************************************************/
//2018 04 21 Kaoru Ota
//
/****************************************************************************************/

bool Judgment::det_area(float x_left, float y_under, float x_right, float y_top, float x_value, float y_value){
  if(x_left < x_value && x_value <= x_right && y_under < y_value && y_value <= y_top){
    return true;
  }else{
    return false;
  }
}

/****************************************************************************************/
//2018 11 07 Kaoru Ota
//it has not been completed, it do not work well.
/****************************************************************************************/
void Judgment::det_on_line(){

}



void Judgment::setEyeCommand(int     linevalue,
			      bool    green_flag,
			      float   xvalue,
			      float   yvalue,
			      float   pre_50mm_x, //20180512 kota
			      float   pre_50mm_y,
			      float   odo,
			      float   velocity,
                              float   pre_velo_0p5sec,
			      float   yawrate,
			      float   abs_angle,
			      float   ave_angle,
			      int     robo_tail_angle,
			      bool    robo_stop,
			      bool    robo_forward,
			      bool    robo_back,
			      bool    robo_turn_left,
			      bool    robo_turn_right,
                              int16_t sonar_dis){

  mLinevalue       = linevalue;
  mGreen_flag      = green_flag;
  mXvalue          = xvalue;
  mYvalue          = yvalue;
  mPre_50mm_x      = pre_50mm_x;//50mm saki 20180512 kota
  mPre_50mm_y      = pre_50mm_y;//50mm saki 20180512 kota
  mOdo             = odo; 
  mVelocity        = velocity;
  mPre_velo_0p5sec = pre_velo_0p5sec;
  mYawrate         = yawrate;
  mYawangle        = abs_angle;
  mAve_yaw_angle   = ave_angle;

  mTail_angle      = robo_tail_angle;
  mRobo_stop       = robo_stop;
  mRobo_forward    = robo_forward;
  mRobo_back       = robo_back;
  mRobo_turn_left  = robo_turn_left;
  mRobo_turn_right = robo_turn_right;
  mSonar_dis       = sonar_dis;
}


void Judgment::GetCalcResult(int   forward_calc,
			      float yawratecmd_calc,
			      float ref_tail_angle_calc,
			      bool  tail_stand_mode_calc){
  
  forward         = forward_calc;
  yawratecmd      = yawratecmd_calc;
  ref_tail_angle  = ref_tail_angle_calc;
  tail_stand_mode = tail_stand_mode_calc;

}

