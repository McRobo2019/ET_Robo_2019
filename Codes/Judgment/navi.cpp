
/******************************************************************************
 *  navi.cpp
*****************************************************************************/
#include <stdlib.h>
#include "ev3api.h"
#include "navi.hpp"
#include "math.h"

Navi::Navi() {

}

void Navi::init() {
  ZONE            = START_ZONE;
  target_velocity = 10;
  lost_line       = false;
  gAve_yaw_angle_500->init(); //20181108
  gAve_x_500->init();
  gAve_y_500->init();

}


float Navi::omega_frm_vector(float target_x, float target_y, float current_x, float current_y, float yaw_angle, int velocity){

  float x12, y12, x10, y10;
  float predic_dist;
  float a;
  float angle_vec0_vec1;
  float dist_vec0;
  float time;
  float ref_omega;

  predic_dist = 0.5 * (float)velocity; //0.5 sec x velocity;

  //vector 0
  x12 = target_x - current_x;
  y12 = target_y - current_y;

  //vector 1
  x10 = predic_dist * cos(yaw_angle);
  y10 = predic_dist * sin(yaw_angle);

  a   = (x12*y10)-(y12*x10);
  
  dist_vec0 = sqrt(pow(x12,2.0) + pow(y12,2.0));

  angle_vec0_vec1 = a/dist_vec0;

  time = dist_vec0/(float)velocity;

  ref_omega = -1.0 * angle_vec0_vec1/time * RAD_1_DEG;

  return ref_omega;

}

float Navi::omega_frm_circle(float circle_x, float circle_y, float circle_r, float current_x, float current_y, float yaw_angle, int velocity){

  float x10, y10;
  float predic_dist;
  float x0,y0,a,a2,b,b2,r2;
  float diff_r;
  float ref_omega;

  predic_dist = 0.5 * (float)velocity; //0.5 sec x velocity;


  //vector
  x10 = predic_dist * cos(yaw_angle);
  y10 = predic_dist * sin(yaw_angle);

  x0  = current_x + x10;
  y0  = current_y + y10;

  a   = circle_x - x0;
  b   = circle_y - y0;

  a2  = a * a;
  b2  = b * b;
  r2  = a2 + b2;

  if(circle_r < 0){
    diff_r = -1.0 * circle_r - sqrt(r2);
  }else{
    diff_r = sqrt(r2) - circle_r;
  }

  if(diff_r > 45.0) diff_r = 45.0;
  if(diff_r < -45.0) diff_r = -45.0;
   
  ref_omega = diff_r * RAD_1_DEG;

  return ref_omega;
}


/****************************************************************************************/
//void Navi::run(int line_val, int odo, int velocity, float yaw_angle, float ave_yaw_angle, int x, int y, int pre_50mm_x, int pre_50mm_y) {
void Navi::run(int line_val, int odo, int velocity, float yaw_angle, int x, int y, int pre_50mm_x, int pre_50mm_y) {
  static float ref_odo;
  static float dif_odo;

  static int   ref_velocity;
  static float acl_velocity;
  static int   max_y;


  switch(ZONE){

    /** LEFT 2019 NEW CONCEPT **********************************************************************/      
  case START_ZONE:
    LOG_NAVI = 1000;

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
    min_omega = MINUS_RAD_22P5_DEG;
    ref_omega = 0;
    max_omega = RAD_22P5_DEG;

    //-------------------------------------------------------------REF YAW RATE GEN

    //LINE TRACE-------------------------------------------------------------------------------
    target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);
    //--------------------------------------------------------------------------------LINE TRACE



    //DET RUNNING ARE-------------------------------------------------------------
    if (pre_50mm_x > FIRST_STRAIGHT_AREA[0]){
      ZONE = FIRST_STRAIGHT_ZONE;

      //FOR TARGET_VELOCITY GEN--------------------------------------------------------
      ref_velocity = target_velocity;
      ref_odo      = odo;
      //--------------------------------------------------------FOR TARGET_VELOCITY GEN
    }

    break;

/** LEFT 2019 ***********************************************************************/
    case FIRST_STRAIGHT_ZONE:
      LOG_NAVI = 1020;

      //TARGET_VELOCITY GEN--------------------------------------------------------------
      dif_odo = odo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity    = ref_velocity + (dif_odo * ACCEL_GAIN);
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

      //CORRECT Y & YAW ANGLE-------------------------------------------------------------
      if( (line_val > 40) && (line_val < 60) ){
	Y_POS         = 165.0;
	ave_yaw_angle =gAve_yaw_angle_500->average_500(yaw_angle);
      }
      
      //LINE TRACE-------------------------------------------------------------------------------
      target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);
      //--------------------------------------------------------------------------------LINE TRACE


      //DET RUNNING ARE-------------------------------------------------------------
      if (pre_50mm_x > ENTER_1ST_CORNER_AREA[0]){

	//CORRECT Y YAW ANGLE-------------------------------------------------------------
	//	Y_POS            = 165.0;
	YAW_ANGLE_OFFSET = 0.0 - ave_yaw_angle;

	gAve_yaw_angle_500->init(); //20181108
	gAve_x_500->init();
	gAve_y_500->init();


	ZONE = ENTER_1ST_CORNER_ZONE;
	//FOR TARGET_VELOCITY GEN--------------------------------------------------------
	ref_velocity = target_velocity;
	ref_odo     = odo;
	//--------------------------------------------------------FOR TARGET_VELOCITY GEN
      }
      break;


/** LEFT 2019 ***********************************************************************/
    case ENTER_1ST_CORNER_ZONE:
      LOG_NAVI = 1030;
      dif_odo = odo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > ENTER_1ST_CORNER_VELOCITY_VAL){
	target_velocity =  ENTER_1ST_CORNER_VELOCITY_VAL;
      }

      min_omega = MINUS_RAD_15_DEG;
      ref_omega = 0.0;
      max_omega = RAD_15_DEG;

      //LINE TRACE-------------------------------------------------------------------------------
      target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);
      //--------------------------------------------------------------------------------LINE TRACE

      if (pre_50mm_x >FIRST_CORNER_AREA[0]){
	ZONE = FIRST_CORNER_ZONE;
      }
      break;

/** LEFT 2019 ***********************************************************************/
    case FIRST_CORNER_ZONE:
      LOG_NAVI = 1040;

      target_velocity =  FIRST_CORNER_VELOCITY_VAL;
      target_omega = omega_frm_circle(CIRCLE_01[0], CIRCLE_01[1], CIRCLE_01[2], x, y, yaw_angle, velocity);

      if (pre_50mm_y > SECOND_STRAIGHT_AREA[1]){
	ZONE = SECOND_STRAIGHT_ZONE;
      }
      break;

/** LEFT 2019 ***********************************************************************/
    case SECOND_STRAIGHT_ZONE:
      LOG_NAVI = 1050;

      target_velocity = SECOND_STRAIGHT_VELOCITY_VAL;
      target_omega = omega_frm_vector(STRAIGT_02[0],STRAIGT_02[1], x, y, yaw_angle, velocity);

      if (pre_50mm_y > SECOND_CORNER_AREA_2[1]){
	ZONE = SECOND_CORNER_ZONE;
      }
      break;
      
/** LEFT 2019 ***********************************************************************/
    case SECOND_CORNER_ZONE:
      LOG_NAVI = 1070;

      if(x < CIRCLE_03[0]){
	target_velocity = 100;
	target_omega = omega_frm_circle(CIRCLE_22[0], CIRCLE_22[1], CIRCLE_22[2], x, y, yaw_angle, velocity);
	
      }else if (x < (CIRCLE_03[0] - CIRCLE_03[2])){
	target_velocity = 0;
	target_omega    = 0.0;
      }else{
	target_velocity = SECOND_CORNER_VELOCITY_VAL;
	target_omega = omega_frm_circle(CIRCLE_22[0], CIRCLE_22[1], CIRCLE_22[2], x, y, yaw_angle, velocity);
      }

      
      break;



/** LEFT 2019 ***********************************************************************/
  case THIRD_CORNER_ZONE:
    LOG_NAVI = 1090;

    target_velocity = THIRD_CORNER_VELOCITY_VAL;

    ref_omega = (float)velocity/CIRCLE_03[2];
    //    min_omega = ref_omega - RAD_15_DEG;
    min_omega = MINUS_RAD_5_DEG;
    max_omega = ref_omega + RAD_15_DEG;

    if (y < FOURTH_CORNER_AREA[3]){
      ZONE = FOURTH_CORNER_ZONE;
    }
    break;



/** LEFT 2019 ***********************************************************************/
  case FOURTH_CORNER_ZONE:
    LOG_NAVI = 1110;
    target_velocity = FOURTH_CORNER_VELOCITY_VAL;
    /*
    min_omega = MINUS_RAD_45_DEG;
    ref_omega = 0.0;
    max_omega = RAD_45_DEG;
    */
    
//       if (odo < ref_odo){
//       min_omega = MINUS_RAD_22P5_DEG;
//       ref_omega = 0.0;
//       max_omega = RAD_22P5_DEG;
//     } else {
//     ref_omega= (float)velocity/CIRCLE_04[2];
//     min_omega= ref_omega - RAD_15_DEG;
//     max_omega= ref_omega + RAD_15_DEG;
//     }

    if (y > CIRCLE_04[1]){
      LOG_NAVI = 1111;
      //min_omega = MINUS_RAD_22P5_DEG;
      min_omega= (float)velocity/CIRCLE_04[2] +  MINUS_RAD_30_DEG;
      ref_omega = 0.0;
      max_omega = RAD_22P5_DEG;
    }else{
      LOG_NAVI = 1112;
      ref_omega= (float)velocity/CIRCLE_04[2];
      min_omega= ref_omega - RAD_15_DEG;
      max_omega= ref_omega + RAD_15_DEG;
    }

    if (pre_50mm_x < FIFTH_CORNER_AREA[2]){ZONE = FIFTH_CORNER_ZONE;
    }
    break;

/** LEFT 2019 ***********************************************************************/
      case FIFTH_CORNER_ZONE:
	LOG_NAVI = 1120;
	target_velocity = FIFTH_CORNER_VELOCITY_VAL;

	//	ref_omega = -1.0 * (float)velocity/CIRCLE_05[2];
	ref_omega = (float)velocity/CIRCLE_05[2];
	min_omega = ref_omega - RAD_15_DEG;
	max_omega = ref_omega + RAD_15_DEG;

	if (pre_50mm_y > THIRD_STRAIGHT_AREA[1]){
	  ZONE = THIRD_STRAIGHT_ZONE;
	}
	break;

/** LEFT 2019 ***********************************************************************/
    case THIRD_STRAIGHT_ZONE:
      LOG_NAVI = 1130;
      target_velocity = THIRD_STRAIGHT_VELOCITY_VAL;

      min_omega = MINUS_RAD_5_DEG;
      ref_omega = 0;
      max_omega = RAD_5_DEG;



      if (pre_50mm_y > SIXTH_CORNER_AREA[1]){
	ZONE = SIXTH_CORNER_ZONE;
      }
      break;
	
/** LEFT 2019 ***********************************************************************/
      case SIXTH_CORNER_ZONE:
	LOG_NAVI = 1140;
	target_velocity = SIXTH_CORNER_VELOCITY_VAL;

	//	ref_omega = -1.0 * (float)velocity/CIRCLE_06[2];
	ref_omega = (float)velocity/CIRCLE_06[2];
	min_omega = ref_omega - RAD_22P5_DEG;
	max_omega = ref_omega + RAD_15_DEG;




	if (x > FOURTH_STRAIGHT_AREA[0]){
	  ZONE = FOURTH_STRAIGHT_ZONE;
	  ref_velocity = target_velocity;
	  ref_odo      = odo;
	  gAve_yaw_angle_500->init();
	}
	break;

/** LEFT 2019 ***********************************************************************/
    case FOURTH_STRAIGHT_ZONE:
      LOG_NAVI = 1200;

      dif_odo = odo - ref_odo;
      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > FOURTH_STRAIGHT_VELOCITY_VAL){
	target_velocity = FOURTH_STRAIGHT_VELOCITY_VAL;
      }

      min_omega = MINUS_RAD_5_DEG;
      ref_omega = 0;
      max_omega = RAD_5_DEG;

      if( (line_val > 40) && (line_val < 60) ){
	Y_POS         = 1625.0;
	ave_yaw_angle =gAve_yaw_angle_500->average_500(yaw_angle);
      }


      if (pre_50mm_x > SEVENTH_CORNER_AREA[0]){
	ZONE =     SEVENTH_CORNER_ZONE;
	ref_velocity = target_velocity;
	ref_odo      = odo;
	YAW_ANGLE_OFFSET = 0.0 - ave_yaw_angle;

      }
      break;


/** LEFT 2019 ***********************************************************************/
      case SEVENTH_CORNER_ZONE:
	LOG_NAVI = 1210;
	target_velocity = SEVENTH_CORNER_VELOCITY_VAL;

	//	ref_omega = -1.0 * (float)velocity/CIRCLE_07[2];
	ref_omega = (float)velocity/CIRCLE_07[2];
	min_omega = ref_omega - RAD_15_DEG;
	max_omega = ref_omega + RAD_15_DEG;

	//straight
	if(y < CIRCLE_07[1]){
	  min_omega = MINUS_RAD_22P5_DEG;
	  ref_omega = 0;
	  max_omega = RAD_22P5_DEG;
	}

	//	if (pre_50mm_y < EIGHTH_CORNER_AREA[3]){
	if (y < EIGHTH_CORNER_AREA[3]){
	  ZONE = EIGHTH_CORNER_ZONE;
	  ref_velocity = target_velocity;
	  ref_odo      = odo;
	}
	break;



/** LEFT 2019 ***********************************************************************/
      case EIGHTH_CORNER_ZONE:
	LOG_NAVI = 1220;
	target_velocity = EIGHTH_CORNER_VELOCITY_VAL;

	//	ref_omega = -1.0 * (float)velocity/CIRCLE_08[2];
	ref_omega = (float)velocity/CIRCLE_08[2];
	min_omega = ref_omega - RAD_15_DEG;
	max_omega = ref_omega + RAD_15_DEG;

	if (y < NINTH_CORNER_AREA[3]){
	  ZONE = NINTH_CORNER_ZONE;
	  ref_velocity = target_velocity;
	  ref_odo      = odo;
	}
	break;


/** LEFT 2019 ***********************************************************************/
  case NINTH_CORNER_ZONE:
    LOG_NAVI = 1230;
    target_velocity = NINTH_CORNER_VELOCITY_VAL;


    if(y > CIRCLE_09[1]){
      LOG_NAVI = 1231;
      /*
      min_omega = MINUS_RAD_22P5_DEG;
      ref_omega = 0;
      max_omega = RAD_22P5_DEG;
      */
      /*
      min_omega = MINUS_RAD_5_DEG;
      ref_omega = 0;
      max_omega = RAD_22P5_DEG;
      */


      min_omega = MINUS_RAD_15_DEG;
      ref_omega = 0;
      max_omega = RAD_45_DEG;

      /*
      min_omega = MINUS_RAD_22P5_DEG;
      ref_omega = 0;
      max_omega = RAD_90_DEG;
      */


    }else{
      LOG_NAVI = 1232;      
      ref_omega = (float)velocity/CIRCLE_09[2];
      min_omega = ref_omega - RAD_15_DEG;
      max_omega = ref_omega + RAD_15_DEG;
      
    }
    
    if ((pre_50mm_x > TENTH_CORNER_AREA[0])&&(y < CIRCLE_09[1])){
      ZONE = TENTH_CORNER_ZONE;
      ref_velocity = target_velocity;
      ref_odo      = odo;
    }
    break;

/** LEFT 2019 ***********************************************************************/
      case TENTH_CORNER_ZONE:
	LOG_NAVI = 1240;
	target_velocity = TENTH_CORNER_VELOCITY_VAL;

	ref_omega = (float)velocity/CIRCLE_10[2];
	min_omega = ref_omega - RAD_15_DEG;
	max_omega = ref_omega + RAD_15_DEG;

	if (y > FIFTH_STRAIGHT_AREA[1]){
	  ZONE =     FIFTH_STRAIGHT_ZONE;
	  ref_velocity = target_velocity;
	  ref_odo     = odo;
	}
	break;

    case FIFTH_STRAIGHT_ZONE:
      LOG_NAVI = 1050;

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

      break;



/** LEFT 2019 ***********************************************************************/
    case LUG_ZONE:
      LOG_NAVI = 1130;
      target_velocity = 0;
      break;


/** LEFT 2019 ***********************************************************************/
    case SECOND_GRAY_ZONE:
      LOG_NAVI = 1140;
      break;

/** LEFT 2019 ***********************************************************************/
    case GARAGE_ZONE:
      LOG_NAVI = 1150;
      break;

/** LEFT 2019 ***********************************************************************/
    case LOST:
      LOG_NAVI = 1160;
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





