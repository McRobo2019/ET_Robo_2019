/******************************************************************************
 *  navi.cpp
*****************************************************************************/
#include <stdlib.h>
#include "ev3api.h"
#include "navi.hpp"


Navi::Navi() {

}

void Navi::init() {
  ZONE            = START_ZONE;
  target_velocity = 10;
  det_navi_log    = 0;
  lost_line       = false;
  gAve_yaw_angle_500->init(); //20181108
}


/****************************************************************************************/
/****************************************************************************************/
void Navi::run(int odo, int velocity, float yaw_angle, float ave_yaw_angle, int x, int y, int pre_50mm_x, int pre_50mm_y) {

  float        yaw_time;
  static float ref_odo;
  static float dif_odo;

  static int   ref_velocity;
  static float acl_velocity;

  ave_yaw_angle_500 = gAve_yaw_angle_500->average_500(yaw_angle);

  switch(ZONE){

    /** LEFT 2019 ***********************************************************************/      
  case START_ZONE:
    det_navi_log = 1000;

    //TARGET_VELOCITY GEN--------------------------------------------------------------
    ref_odo = odo;

    if(ref_odo < 0){
      ref_odo = 0;
    }

    acl_velocity = 10 + (ref_odo * ACCEL_GAIN);
    target_velocity = (int)acl_velocity;

    if(target_velocity > START_VELOCITY_VAL){
      target_velocity = START_VELOCITY_VAL;
    }
    //--------------------------------------------------------------TARGET_VELOCITY GEN

    //REF YAW RATE GEN-------------------------------------------------------------
    if(velocity > 10){
      min_omega = MINUS_RAD_22P5_DEG;
      ref_omega = 0;
      max_omega = RAD_22P5_DEG;
    }else{
      min_omega = 0;
      ref_omega = 0;
      max_omega = 0;
    }
    //-------------------------------------------------------------REF YAW RATE GEN


    //DET RUNNING ARE-------------------------------------------------------------
    if (pre_50mm_x > FIRST_STRAIGHT_AREA[0]){
      
      ZONE = FIRST_STRAIGHT_ZONE;

      //FOR TARGET_VELOCITY GEN--------------------------------------------------------
      ref_velocity = target_velocity;
      ref_odo     = odo;
      //--------------------------------------------------------FOR TARGET_VELOCITY GEN

      //DET START FAIL
    }else if(x < START_AREA[0]){
      ZONE = START_BACK;

    }else if ((ave_yaw_angle < MINUS_RAD_22P5_DEG)||(ave_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE
      lost_line       = true;
      target_velocity = 0;
      min_omega       = 0.0;
      max_omega       = 0.0;
    }
    break;


/** LEFT 2019 ***********************************************************************/
    case START_BACK:
      det_navi_log = 1010;
      target_velocity = 70;
      //DET RE-START
      if(x > X_POS_OFFSET ){
	ZONE = START_ZONE;
      }
      break;


/** LEFT 2019 ***********************************************************************/
    case FIRST_STRAIGHT_ZONE:
      det_navi_log = 1020;

      //TARGET_VELOCITY GEN--------------------------------------------------------------
      dif_odo = odo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > FIRST_STRAIGHT_VELOCITY_VAL){
	target_velocity =  FIRST_STRAIGHT_VELOCITY_VAL;
      }
      //--------------------------------------------------------------TARGET_VELOCITY GEN

      //REF YAW RATE GEN-------------------------------------------------------------
      min_omega = MINUS_RAD_22P5_DEG;
      ref_omega = 0;
      max_omega = RAD_22P5_DEG;
      //-------------------------------------------------------------REF YAW RATE GEN

      //DET RUNNING ARE-------------------------------------------------------------
      if (pre_50mm_x > ENTER_1ST_CORNER_AREA[0]){
	ZONE = ENTER_1ST_CORNER_ZONE;
	//FOR TARGET_VELOCITY GEN--------------------------------------------------------
	ref_velocity = target_velocity;
	ref_odo     = odo;
	//--------------------------------------------------------FOR TARGET_VELOCITY GEN

      //DET LINE_LOST ARE-------------------------------------------------------------
      }else if( (ave_yaw_angle_500 < MINUS_RAD_22P5_DEG)|| (y < (STRAIGT_01[1]- 150)) ){
	lost_line       = true;
	target_velocity = 0;
	min_omega = 0.0;
	max_omega = 0.0;
      }
      break;


/** LEFT 2019 ***********************************************************************/
    case ENTER_1ST_CORNER_ZONE:
      det_navi_log = 1030;
      dif_odo = odo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > ENTER_1ST_CORNER_VELOCITY_VAL){
	target_velocity =  ENTER_1ST_CORNER_VELOCITY_VAL;
      }

      min_omega = -1.0 * RAD_6_DEG;
      ref_omega = 0.0;
      yaw_time     = CIRCLE_01_LENGTH/velocity;
      max_omega = (CIRCLE_01_ANGLE/yaw_time) + RAD_22P5_DEG;

      if (pre_50mm_x >FIRST_CORNER_AREA[0]){
	ZONE = FIRST_CORNER_ZONE;
	ref_velocity = target_velocity;
	ref_odo     = odo;
      }else if ((ave_yaw_angle < MINUS_RAD_22P5_DEG)||(ave_yaw_angle > RAD_45_DEG)){ //LOST THE LINE
	lost_line       = true;
	target_velocity = 0;
	min_omega       = 0.0;
	max_omega       = 0.0;
      }
      break;

/** LEFT 2019 ***********************************************************************/
    case FIRST_CORNER_ZONE:
      det_navi_log = 1040;

      dif_odo = odo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > FIRST_CORNER_VELOCITY_VAL){
	target_velocity =  FIRST_CORNER_VELOCITY_VAL;
      }

      yaw_time  = CIRCLE_01_LENGTH/velocity;
      ref_omega = CIRCLE_01_ANGLE/yaw_time;
      min_omega = ref_omega - RAD_22P5_DEG;
      max_omega = ref_omega + RAD_45_DEG;

      if (pre_50mm_y > SECOND_STRAIGHT_AREA[1]){
	ZONE = SECOND_STRAIGHT_ZONE;

	ref_velocity = target_velocity;
	ref_odo     = odo;

      }else if ((ave_yaw_angle < MINUS_RAD_22P5_DEG)||(ave_yaw_angle > RAD_120_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	lost_line       = true;
	target_velocity = 0;
	min_omega       = 0.0;
	max_omega       = 0.0;
      }
      break;

/** LEFT 2019 ***********************************************************************/
    case SECOND_STRAIGHT_ZONE:
      det_navi_log = 1050;

      dif_odo = odo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > SECOND_STRAIGHT_VELOCITY_VAL){
	target_velocity = SECOND_STRAIGHT_VELOCITY_VAL;
      }

      min_omega = MINUS_RAD_22P5_DEG;
      ref_omega = 0;
      max_omega = RAD_22P5_DEG;


      if (pre_50mm_y > ENTER_2ND_CORNER_AREA[1]){

      	ZONE = ENTER_2ND_CORNER_ZONE;
	ref_velocity = target_velocity;
	ref_odo     = odo;
      }else if ((ave_yaw_angle <  RAD_45_DEG) || (ave_yaw_angle > RAD_120_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	lost_line       = true;
	target_velocity = 0;
	min_omega       = 0.0;
	max_omega       = 0.0;
      }
      break;
      
/** LEFT 2019 ***********************************************************************/
    case ENTER_2ND_CORNER_ZONE:
      det_navi_log = 1060;

      dif_odo = odo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > ENTER_2ND_CORNER_VELOCITY_VAL){
	target_velocity = ENTER_2ND_CORNER_VELOCITY_VAL;
      }

      yaw_time     = CIRCLE_02_LENGTH/velocity;
      min_omega = (CIRCLE_02_ANGLE/yaw_time) - RAD_22P5_DEG;
      ref_omega = 0.0;
      max_omega = RAD_22P5_DEG;

      if (pre_50mm_y > SECOND_CORNER_AREA[1]){
	ZONE = SECOND_CORNER_ZONE;
	ref_velocity = target_velocity;
	ref_odo     = odo;
      }else if ((ave_yaw_angle <  RAD_45_DEG) || (ave_yaw_angle > RAD_120_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	lost_line       = true;
	target_velocity = 0;
	min_omega       = 0.0;
	max_omega       = 0.0;
      }
      break;

/** LEFT 2019 ***********************************************************************/
    case SECOND_CORNER_ZONE:
      det_navi_log = 1070;
      dif_odo = odo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > SECOND_CORNER_VELOCITY_VAL){
	target_velocity = SECOND_CORNER_VELOCITY_VAL;
      }
      yaw_time     = CIRCLE_02_LENGTH/velocity;
      ref_omega = CIRCLE_02_ANGLE/yaw_time;

      min_omega = ref_omega - RAD_22P5_DEG;
      ref_omega = 0;
      max_omega = ref_omega + RAD_22P5_DEG;

      //      if (pre_50mm_x > THIRD_STRAIGHT_AREA[0]){
      //	ZONE = THIRD_STRAIGHT_ZONE;
      if (pre_50mm_x < THIRD_CORNER_AREA[2]){
	ZONE = THIRD_CORNER_ZONE;
	ref_velocity = target_velocity;
	ref_odo     = odo;
      }else if ((ave_yaw_angle < MINUS_RAD_22P5_DEG) || (ave_yaw_angle > RAD_120_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	lost_line       = true;
	target_velocity = 0;
	min_omega       = 0.0;
	max_omega       = 0.0;
      }
      break;

/** LEFT 2019 ***********************************************************************/
    case THIRD_STRAIGHT_ZONE:
      det_navi_log = 1080;
      dif_odo = odo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > THIRD_STRAIGHT_VELOCITY_VAL){
	target_velocity = THIRD_STRAIGHT_VELOCITY_VAL;
      }

      yaw_time     = CIRCLE_02_LENGTH/velocity;
      min_omega = CIRCLE_02_ANGLE/yaw_time;

      ref_omega = 0;
      max_omega = RAD_22P5_DEG;

      if (pre_50mm_x > THIRD_CORNER_AREA[0]){

	ZONE = THIRD_CORNER_ZONE;
	ref_velocity = target_velocity;
	ref_odo     = odo;
      }else if ((ave_yaw_angle < MINUS_RAD_22P5_DEG) || (ave_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	lost_line       = true;
	target_velocity = 0;
	min_omega       = 0.0;
	max_omega       = 0.0;
      }
      break;


/** LEFT 2019 ***********************************************************************/
    case THIRD_CORNER_ZONE:
      det_navi_log = 1090;

      dif_odo = odo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > THIRD_CORNER_VELOCITY_VAL){
	target_velocity = THIRD_CORNER_VELOCITY_VAL;
      }

      yaw_time     = CIRCLE_03_LENGTH/velocity;
      ref_omega = CIRCLE_03_ANGLE/yaw_time;
      
      min_omega = ref_omega - RAD_22P5_DEG;
      ref_omega = 0;
      max_omega = ref_omega + RAD_22P5_DEG;

      //      if (pre_50mm_y < (FOURTH_STRAIGHT_AREA[3] - 110 )){
      //	ZONE = FOURTH_STRAIGHT_ZONE;
      if (pre_50mm_y < FOURTH_CORNER_AREA[3]){

	ref_velocity = target_velocity;
	ref_odo     = odo;
      }else if ((ave_yaw_angle < MINUS_RAD_145_DEG ) || (ave_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	lost_line       = true;
	target_velocity = 0;
	min_omega       = 0.0;
	max_omega       = 0.0;
      }
      break;


/** LEFT 2019 ***********************************************************************/
    case FOURTH_STRAIGHT_ZONE:
      det_navi_log = 1100;

      dif_odo = odo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > FOURTH_STRAIGHT_VELOCITY_VAL){
	target_velocity = FOURTH_STRAIGHT_VELOCITY_VAL;
      }

      if (pre_50mm_y < FOURTH_CORNER_AREA[3] + 300){
	target_velocity = GOAL_VAL;
      }

      min_omega = MINUS_RAD_22P5_DEG;
      ref_omega = 0;
      max_omega = RAD_22P5_DEG;

      if (pre_50mm_y < FOURTH_CORNER_AREA[3]){
	ZONE = FOURTH_CORNER_ZONE;
	ref_velocity = target_velocity;
	ref_odo     = odo;
      }else if ((ave_yaw_angle < MINUS_RAD_145_DEG ) || (ave_yaw_angle > MINUS_RAD_90_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	lost_line       = true;
	target_velocity = 0;
	min_omega       = 0.0;
	max_omega       = 0.0;
      }
      break;


/** LEFT 2019 ***********************************************************************/
      case FOURTH_CORNER_ZONE:
	det_navi_log = 1110;
	target_velocity = FOURTH_CORNER_VELOCITY_VAL;
	yaw_time     = CIRCLE_04_LENGTH/300;
	ref_omega = CIRCLE_04_ANGLE/yaw_time;
	max_omega = ref_omega + RAD_22P5_DEG;
	ref_omega = 0;

	if(ave_yaw_angle > 0){
	  min_omega = MINUS_RAD_5_DEG;
	}else{
	  min_omega = 0;
	}

	if (  (x > FIRST_GRAY_AREA[0])&&(y < FIRST_GRAY_AREA[3])){
	  ZONE = LUG_ZONE;
	  ref_velocity = target_velocity;
	  ref_odo     = odo;
	}else if ((ave_yaw_angle < MINUS_RAD_145_DEG ) || (ave_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE -> MAP TRACE MODE
	  lost_line       = true;
	  target_velocity = 0;
	  min_omega       = 0.0;
	  max_omega       = 0.0;
	}
	break;


/** LEFT 2019 ***********************************************************************/
    case FIRST_GRAY_ZONE:
      det_navi_log = 1120;
      target_velocity = FIRST_GRAY_VELOCITY_VAL;

      min_omega = MINUS_RAD_5_DEG;
      ref_omega = 0;
      max_omega = RAD_5_DEG;

      if (pre_50mm_x > LUG_AREA[0]){
	ZONE = LUG_ZONE;
      }else if ((ave_yaw_angle < MINUS_RAD_22P5_DEG)||(ave_yaw_angle > RAD_22P5_DEG)){ //LOST THE LINE
	lost_line       = true;
	target_velocity = 0;
	min_omega       = 0.0;
	max_omega       = 0.0;
      }
      break;


/** LEFT 2019 ***********************************************************************/
    case LUG_ZONE:
      det_navi_log = 1130;
      target_velocity = 0;
      break;


/** LEFT 2019 ***********************************************************************/
    case SECOND_GRAY_ZONE:
      det_navi_log = 1140;
      break;

/** LEFT 2019 ***********************************************************************/
    case GARAGE_ZONE:
      det_navi_log = 1150;
      break;

/** LEFT 2019 ***********************************************************************/
    case LOST:
      det_navi_log = 1160;
      target_velocity = 0;
      ref_omega = 0;
      lost_line       = true;
      target_velocity = 0;
      min_omega       = 0.0;
      max_omega       = 0.0;
      break;

    default:
      break;
    }
}



