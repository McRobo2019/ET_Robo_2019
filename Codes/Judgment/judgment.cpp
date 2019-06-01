/******************************************************************************
 *  Author: Koaru Ota

--- basic structure ---

*****************************************************************************/

#include <stdlib.h>
#include "ev3api.h"
#include "judgment.hpp"
#include "Clock.h"

using ev3api::Clock;
Clock*       Jud_Clock;


Judgment::Judgment() {

}

void Judgment::init() {

  Jud_Clock       = new Clock();        

  ZONE               = START_ZONE;
  DRIVE_MODE         = LINE_TRACE;
  ON_LINE_MODE       = ON_THE_LEFT_EDGE;
  PRE_ON_LINE_MODE   = ON_THE_LEFT_EDGE;
  TEST_MODE          = MODE_00;

  on_line            = true;
  left_line          = false;
  right_line         = false;
  lost_line          = false;
  line_to_map        = false; //181108

  line_trace_mode    = true;

  gAve_line_val->init();
  gAve_yaw_angle_500->init(); //20181108

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

}

/****************************************************************************************/
//2018 04 21 Kaoru Ota
//    //https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo
/****************************************************************************************/
void Judgment::det_navigation() {

  //  float yaw_time;

  static float ref_odo;
  static float ref_angle;
  /*
  static float dif_odo;

  static int   ref_forward;
  static float acl_forward;
  */
  static uint ref_clock;
  static float dir_mode;
  
  if(DRIVE_MODE == LINE_TRACE){

    line_trace_mode    = false;
    switch(TEST_MODE){
    case MODE_00: // set init value
      forward         = 0;
      target_yaw_rate = 0.0;
      dir_mode  = 1.0;
      ref_clock = Jud_Clock->now() + 1000; //1sec
      ref_odo   = mOdo + 1500; // 3m
      TEST_MODE = MODE_01;
      break;

    case MODE_01: //start 
      forward         = 0;
      target_yaw_rate = 0.0;
      if(Jud_Clock->now() > ref_clock){
	TEST_MODE = MODE_02;
      }
      break;

    case MODE_02:  //Search
      forward         = 0;
      target_yaw_rate = dir_mode * RAD_45_DEG;
      if( mSonar_dis > 120){	
	TEST_MODE = MODE_03; //forward
	ref_odo   = mOdo + 3000; // 3m
	dir_mode = -1.0 * dir_mode;
      }
      break;
      
    case MODE_03:  //forward
      forward         = 60;
      target_yaw_rate = 0.0;

      //      if( (mOdo > ref_odo) || (mSonar_dis <50)){

      if( mSonar_dis <50){
	TEST_MODE = MODE_02; //Search
      }else if( mSonar_dis <90){
	TEST_MODE = MODE_05;
	ref_angle = mYawangle + RAD_45_DEG;
      }

      if( mOdo > ref_odo){ //Turn 45
	TEST_MODE = MODE_05;
	ref_angle = mYawangle + RAD_45_DEG;
      }

      if(mAve_wheel_load > 20){
	TEST_MODE = MODE_04;
	ref_odo = mOdo - 150;
      }
      break;

    case MODE_04: //Back
      forward         = -30;
      target_yaw_rate = 0.0;

      if(mOdo <  ref_odo){
	TEST_MODE = MODE_02; //Search
      }
      break;

    case MODE_05: //Turn 5
      forward         = 60;
      target_yaw_rate = RAD_5_DEG;
      
      if(mYawangle > ref_angle){
	TEST_MODE = MODE_06;
	ref_angle = mYawangle - RAD_45_DEG;
      }

      if( mSonar_dis <50){
	TEST_MODE = MODE_02; //Search
      }
      break;

    case MODE_06: //Turn -45
      forward         = 60;
      target_yaw_rate = -1.0*RAD_5_DEG;
      
      if(mYawangle < ref_angle){
	TEST_MODE = MODE_05;
	ref_angle = mYawangle + RAD_45_DEG;
      }

      if( mSonar_dis <50){
	TEST_MODE = MODE_02; //Search
      }
      break;



    default:
      break;
    }



    
  }
  else if(DRIVE_MODE == TRACK){
    line_trace_mode    = false;

    line_trace_mode    = false;
    switch(TEST_MODE){
    case MODE_00: // set init value
      forward         = 0;
      target_yaw_rate = 0.0;
      dir_mode  = 1.0;
      ref_clock = Jud_Clock->now() + 1000; //1sec
      ref_odo   = mOdo + 1500; // 3m
      TEST_MODE = MODE_01;
      break;

    case MODE_01: //start 
      forward         = 0;
      target_yaw_rate = 0.0;
      if(Jud_Clock->now() > ref_clock){
	TEST_MODE = MODE_02;
      }
      break;

    case MODE_02:  //Search
      forward         = 0;
      target_yaw_rate = dir_mode * RAD_45_DEG;
      if( mSonar_dis > 120){	
	TEST_MODE = MODE_03; //forward
	ref_odo   = mOdo + 3000; // 3m
	dir_mode = -1.0 * dir_mode;
      }
      break;
      
    case MODE_03:  //forward
      forward         = 60;
      target_yaw_rate = 0.0;

      //      if( (mOdo > ref_odo) || (mSonar_dis <50)){

      if( mSonar_dis <50){
	TEST_MODE = MODE_02; //Search
      }else if( mSonar_dis <90){
	TEST_MODE = MODE_05;
	ref_angle = mYawangle + RAD_45_DEG;
      }

      if( mOdo > ref_odo){ //Turn 45
	TEST_MODE = MODE_05;
	ref_angle = mYawangle + RAD_45_DEG;
      }

      if(mAve_wheel_load > 20){
	TEST_MODE = MODE_04;
	ref_odo = mOdo - 150;
      }
      break;

    case MODE_04: //Back
      forward         = -30;
      target_yaw_rate = 0.0;

      if(mOdo <  ref_odo){
	TEST_MODE = MODE_02; //Search
      }
      break;

    case MODE_05: //Turn 5
      forward         = 60;
      target_yaw_rate = RAD_5_DEG;
      
      if(mYawangle > ref_angle){
	TEST_MODE = MODE_06;
	ref_angle = mYawangle - RAD_45_DEG;
      }

      if( mSonar_dis <50){
	TEST_MODE = MODE_02; //Search
      }
      break;

    case MODE_06: //Turn -45
      forward         = 60;
      target_yaw_rate = -1.0*RAD_5_DEG;
      
      if(mYawangle < ref_angle){
	TEST_MODE = MODE_05;
	ref_angle = mYawangle + RAD_45_DEG;
      }

      if( mSonar_dis <50){
	TEST_MODE = MODE_02; //Search
      }
      break;



    default:
      break;
    }


    
  }
  else if(DRIVE_MODE == DEBUG){
    line_trace_mode    = false;

    switch(TEST_MODE){
    case MODE_00:
      forward         = 0;
      target_yaw_rate = 0.0;

      ref_clock = Jud_Clock->now() + 1000; //1sec
      ref_odo   = mOdo + 2000; // 3m
      TEST_MODE = MODE_01;
      break;

    case MODE_01:
      forward         = 0;
      target_yaw_rate = 0.0;
      if(Jud_Clock->now() > ref_clock){
	TEST_MODE = MODE_02;
      }
      break;

    case MODE_02:
      forward         = 100;
      target_yaw_rate = 0.0;

      if(mOdo > ref_odo){
	TEST_MODE = MODE_03;
	ref_angle = mYawangle + RAD_180_DEG;
      }
      break;

    case MODE_03:
      	forward         = 100;
	target_yaw_rate = RAD_45_DEG;

	if(mYawangle > ref_angle){
	  TEST_MODE = MODE_02;
	  ref_odo   = mOdo + 2000; // 3m
	}

      break;

    default:
      break;
    }
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
			      float   ave_wheel_load,
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
  mAve_wheel_load  = ave_wheel_load;
  mTail_angle      = robo_tail_angle;
  mRobo_stop       = robo_stop;
  mRobo_forward    = robo_forward;
  mRobo_back       = robo_back;
  mRobo_turn_left  = robo_turn_left;
  mRobo_turn_right = robo_turn_right;
  mSonar_dis       = sonar_dis;
}



