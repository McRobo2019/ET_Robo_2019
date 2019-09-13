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
  det_line        = false;
  det_left_edge   = false;
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

float Navi::omega_frm_angle(float target_angle, float yaw_angle){
  float ref_omega;

  ref_omega = target_angle - yaw_angle;

  if(ref_omega > RAD_30_DEG){
    ref_omega = RAD_30_DEG;
  }else if(ref_omega < MINUS_RAD_30_DEG){
    ref_omega = MINUS_RAD_30_DEG;
  }else{
    ref_omega = 3.0 * ref_omega;
  }
  
  return ref_omega;
}


/****************************************************************************************/
//void Navi::run(int line_val, int odo, int velocity, float yaw_angle, float ave_yaw_angle, int x, int y, int pre_50mm_x, int pre_50mm_y) {
void Navi::run(int line_val, int odo, int velocity, float yaw_angle, int x, int y, int pre_50mm_x, int pre_50mm_y, bool green_flag) {
  static float ref_odo;
  static float dif_odo;
  //  static float ref_x, ref_y;
  //  static float ref_yaw_angle;
  //  static float det_line_angle;
  
  static int   ref_velocity;
  static float acl_velocity;
  //  static int   min_y;
  //  static int   max_y;
  //  float diff_angle;

  switch(ZONE){

    /** LEFT 2019 NEW CONCEPT **********************************************************************/      
  case START_ZONE:
    LOG_NAVI = 1000;

    //TARGET_VELOCITY GEN--------------------------------------------------------------
    /*    ref_odo = odo;

    if(ref_odo < 0){
      ref_odo = 0;
    }

    acl_velocity = 10 + (ref_odo * ACCEL_GAIN);
    target_velocity = (int)acl_velocity;

    if(target_velocity > START_VELOCITY_VAL){
      target_velocity = START_VELOCITY_VAL;
    }
    */
    //--------------------------------------------------------------TARGET_VELOCITY GEN

    target_velocity = START_VELOCITY_VAL;

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
      //ref_velocity = target_velocity;
      //ref_odo      = odo;
      //--------------------------------------------------------FOR TARGET_VELOCITY GEN
    }

    break;

/** LEFT 2019 ***********************************************************************/
    case FIRST_STRAIGHT_ZONE:
      LOG_NAVI = 1020;

      //TARGET_VELOCITY GEN--------------------------------------------------------------
      /*
      dif_odo = odo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity    = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > FIRST_STRAIGHT_VELOCITY_VAL){
	target_velocity =  FIRST_STRAIGHT_VELOCITY_VAL;
      }
      */
      target_velocity =  FIRST_STRAIGHT_VELOCITY_VAL;

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
	//	ref_velocity = target_velocity;
	//	ref_odo     = odo;
	//--------------------------------------------------------FOR TARGET_VELOCITY GEN
      }
      break;

/** LEFT 2019 ***********************************************************************/
    case ENTER_1ST_CORNER_ZONE:
      LOG_NAVI = 1030;

      /*
      dif_odo = odo - ref_odo;

      if(dif_odo < 0){
	dif_odo = 0;
      }

      acl_velocity  = ref_velocity + (dif_odo * ACCEL_GAIN);
      target_velocity = (int)acl_velocity;

      if(target_velocity > ENTER_1ST_CORNER_VELOCITY_VAL){
	target_velocity =  ENTER_1ST_CORNER_VELOCITY_VAL;
      }
      */
      
      target_velocity =  FIRST_STRAIGHT_VELOCITY_VAL;

      min_omega = MINUS_RAD_22P5_DEG;
      ref_omega = 0.0;
      max_omega = RAD_22P5_DEG;

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
      target_omega = omega_frm_vector(STRAIGT_02[2],STRAIGT_02[3], x, y, yaw_angle, velocity);

      if (pre_50mm_y > SECOND_CORNER_AREA_2[1]){
	ZONE       = SECOND_CORNER_ZONE;
	DECEL_GAIN = SECOND_CORNER_VELOCITY_VAL/(RAD_180_DEG - RAD_225_DEG);
      }
      break;
      
/** LEFT 2019 ***********************************************************************/
// Motion Concept 2 , map trace to line trace
/** LEFT 2019 ***********************************************************************/      
    case SECOND_CORNER_ZONE:
      LOG_NAVI = 1070;

      LOG_NAVI        = 1072;
      target_velocity = SECOND_CORNER_VELOCITY_VAL;
      target_omega = omega_frm_circle(CIRCLE_22[0], CIRCLE_22[1], CIRCLE_22[2], x, y, yaw_angle, velocity);

      if(x < CIRCLE_03[0]){
	ZONE  = THIRD_CORNER_ZONE;
      }

      break;

/** LEFT 2019 ***********************************************************************/
  case THIRD_CORNER_ZONE:
    LOG_NAVI = 1090;

    target_velocity = THIRD_CORNER_VELOCITY_VAL;
    target_omega    = omega_frm_circle(CIRCLE_03[0], CIRCLE_03[1], CIRCLE_03[2], x, y, yaw_angle, velocity);

    if(yaw_angle > RAD_270_DEG && line_val > 50){
      det_line = true;
    }
  
    if(det_line){
      LOG_NAVI = 1091;
      if(line_val < 20){
	det_left_edge = true;
	ZONE = FOURTH_CORNER_ZONE;	
      }
    }

    if (y < FOURTH_CORNER_AREA[3]){
      ZONE = FOURTH_CORNER_ZONE;
      //      min_y = 9999;
    }
    break;

/** LEFT 2019 ***********************************************************************/
  case FOURTH_CORNER_ZONE:
    LOG_NAVI = 1110;
    target_velocity = FOURTH_CORNER_VELOCITY_VAL;
    
    if (y > CIRCLE_04[1]){
      LOG_NAVI = 1111;
      min_omega= MINUS_RAD_22P5_DEG;
      ref_omega = 0.0;
      max_omega = RAD_22P5_DEG;

      if(line_val < 20){
	lost_line = true;
      }

      if(lost_line){
	target_velocity = 50;
	if(line_val >45){
	  lost_line = false;
	}
      }
      
    }else{
      LOG_NAVI = 1112;
      ref_omega= (float)velocity/CIRCLE_04[2];
      min_omega= ref_omega - RAD_15_DEG;
      max_omega= ref_omega + RAD_15_DEG;
    }

    //LINE TRACE-------------------------------------------------------------------------------
    target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);
    //--------------------------------------------------------------------------------LINE TRACE

    //For correct y
    /*    if(y < min_y){
      min_y = y;
      }*/

    if (pre_50mm_x < FIFTH_CORNER_AREA[2]){
      ZONE = FIFTH_CORNER_ZONE;
    }
    break;

/** LEFT 2019 ***********************************************************************/
  case FIFTH_CORNER_ZONE:
    LOG_NAVI = 1120;
    target_velocity = FIFTH_CORNER_VELOCITY_VAL;
    ref_omega = (float)velocity/CIRCLE_05[2];
    min_omega = ref_omega - RAD_15_DEG;
    max_omega = ref_omega + RAD_15_DEG;

    //LINE TRACE-------------------------------------------------------------------------------
    target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);
    //--------------------------------------------------------------------------------LINE TRACE

    //For correct y
    /*    if(y < min_y){
      min_y = y;
      }*/
    

    if (pre_50mm_y > THIRD_STRAIGHT_AREA[1]){
      //      Y_POS = Y_POS + (585 - min_y); //koko
      ZONE = THIRD_STRAIGHT_ZONE;
      gAve_yaw_angle_500->init();
    }
    break;
    
/** LEFT 2019 ***********************************************************************/
  case THIRD_STRAIGHT_ZONE:
    LOG_NAVI = 1130;
    target_velocity = THIRD_STRAIGHT_VELOCITY_VAL;

    min_omega = MINUS_RAD_15_DEG;
    ref_omega = 0;
    max_omega = RAD_15_DEG;

    //LINE TRACE-------------------------------------------------------------------------------
    target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);
    //--------------------------------------------------------------------------------LINE TRACE
    
    //CORRECT X YAW ANGLE-------------------------------------------------------------
    if( (line_val > 40) && (line_val < 60) ){
      X_POS  = 130;
      ave_yaw_angle = gAve_yaw_angle_500->average_500(yaw_angle);
      FL_LOG        =	ave_yaw_angle;
    }

    if (pre_50mm_y > SIXTH_CORNER_AREA[1]){
      //CORRECT YAW ANGLE-------------------------------------------------------------
      YAW_ANGLE_OFFSET = RAD_90_DEG - ave_yaw_angle;
      gAve_yaw_angle_500->init();
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
    
    //LINE TRACE-------------------------------------------------------------------------------
    target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);
    //--------------------------------------------------------------------------------LINE TRACE

    if (x > FOURTH_STRAIGHT_AREA[0]){
      ZONE = CORRECT_4TH_ST_ZONE;
      gAve_yaw_angle_500->init();
    }
    break;

/** LEFT 2019 ***********************************************************************/
  case CORRECT_4TH_ST_ZONE:
    LOG_NAVI = 1150;

    target_velocity = CORRECT_4TH_ST_VELOCITY_VAL;
    
    min_omega = MINUS_RAD_15_DEG;
    ref_omega = 0;
    max_omega = RAD_15_DEG;

    //LINE TRACE-------------------------------------------------------------------------------
    target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);
    //--------------------------------------------------------------------------------LINE TRACE

    //CORRECT X YAW ANGLE-------------------------------------------------------------
    if( (line_val > 40) && (line_val < 60) ){
      Y_POS         = 1625.0;
      ave_yaw_angle =gAve_yaw_angle_500->average_500(yaw_angle);
    }
    
    if (x > CIRCLE_22[0]){
      ZONE = FOURTH_STRAIGHT_ZONE;
      //CORRECT YAW ANGLE-------------------------------------------------------------
      YAW_ANGLE_OFFSET = 0.0 - ave_yaw_angle;
      gAve_yaw_angle_500->init();
    }
    break;

  case FOURTH_STRAIGHT_ZONE:
      LOG_NAVI = 1160;

      target_velocity = FOURTH_STRAIGHT_VELOCITY_VAL;
      target_omega = omega_frm_vector(STRAIGT_04[2],STRAIGT_04[3], x, y, yaw_angle, velocity);

      if (pre_50mm_x > SEVENTH_CORNER_AREA[0]){
	ZONE =     SEVENTH_CORNER_ZONE;
      }
      break;
      
      /** LEFT 2019 ***********************************************************************/
  case SEVENTH_CORNER_ZONE:
    LOG_NAVI = 1210;
    target_velocity = SEVENTH_CORNER_VELOCITY_VAL;
    target_omega = omega_frm_circle(CIRCLE_77[0], CIRCLE_77[1], CIRCLE_77[2], x, y, yaw_angle, velocity);
    
    if (y < 800){ //koko
      ZONE =  NINTH_CORNER_ZONE;
    }
    break;

/** LEFT 2019 ***********************************************************************/
  case NINTH_CORNER_ZONE:
    LOG_NAVI = 1230;

    if(yaw_angle < MINUS_RAD_90_DEG){
      target_velocity = 100;
      target_omega = RAD_45_DEG;

    }else{
      target_velocity = NINTH_CORNER_VELOCITY_VAL;
      target_omega = omega_frm_circle(CIRCLE_99[0], CIRCLE_99[1], CIRCLE_99[2], x, y, yaw_angle, velocity);
    }
    
    if(yaw_angle > RAD_90_DEG){
      ZONE = FIFTH_STRAIGHT_ZONE;
    }
    break;


  case FIFTH_STRAIGHT_ZONE:
    LOG_NAVI = 1240;
    target_velocity = FIFTH_STRAIGHT_VELOCITY_VAL;
    target_omega = omega_frm_vector(STRAIGT_05[2],STRAIGT_05[3], x, y, yaw_angle, velocity);

    //    if(pre_50mm_y > STRAIGT_05[3]){
    if(y > CORRECT_5TH_ST_AREA[1]){
      ZONE      = FIND_5TH_ST_LINE;
      FIND_LINE = PRE_FIND;
      ref_odo   = odo + 100;
    }
    break;

  case FIND_5TH_ST_LINE:
    switch(FIND_LINE){
    case PRE_FIND:
      LOG_NAVI = 1300;
      lost_line      = false;
      det_line       = false;
      det_left_edge  = false;
      det_right_edge = false;
      FIND_LINE      = LEFT_SEARCH;
      break;
    
    case LEFT_SEARCH:
      LOG_NAVI = 1310;
      target_velocity = 50;

      if(yaw_angle < RAD_135_DEG){
	target_omega = RAD_22P5_DEG;
      }else{
	target_omega = 0.0;
      }
      
      //DETECT LINE
      if(line_val > 60){
	det_line = true;
      }
      if(det_line){
	LOG_NAVI = 1311;
	if(line_val < 20){
	  lost_line     = true;
	  det_left_edge = true;
	}
      }
      
      if(det_left_edge){
	LOG_NAVI = 1312;
	if(line_val > 50){
	  LOG_NAVI = 1313;
	  FIND_LINE = ON_LINE;
	  ref_odo        = odo + 100;
	}else{
	  target_velocity = 0;
	  target_omega    = MINUS_RAD_22P5_DEG;
	}
      }

      //DETECT GREEN
      if((odo > ref_odo) && (green_flag)){
	LOG_NAVI = 1314;
	target_velocity = 0;
	target_omega    = MINUS_RAD_22P5_DEG;
	if(yaw_angle < RAD_45_DEG){
	  LOG_NAVI = 1315;
	  FIND_LINE = RIGHT_SEARCH;
	  lost_line      = false;
	  det_line       = false;
	  det_left_edge  = false;
	  det_right_edge = false;
	  ref_odo        = odo + 100;
	}
      }
      break;

    case RIGHT_SEARCH:
      LOG_NAVI = 1320;
      target_velocity = 50;

      if(yaw_angle > RAD_45_DEG){
	target_omega = MINUS_RAD_22P5_DEG;
      }else{
	target_omega = 0.0;
      }
      
      //DETECT LINE
      if(line_val > 50){
	LOG_NAVI = 1321;
	det_line       = true;
	det_left_edge  = true;
	FIND_LINE = ON_LINE;
	ref_odo        = odo + 100;	
      }


      //DETECT GREEN
      if((odo > ref_odo) && (green_flag)){
	LOG_NAVI = 1322;
	target_velocity = 0;
	target_omega    = RAD_22P5_DEG;
	if(yaw_angle > RAD_45_DEG){
	  FIND_LINE = LEFT_SEARCH;
	  lost_line      = false;
	  det_line       = false;
	  det_left_edge  = false;
	  det_right_edge = false;
	  ref_odo        = odo + 100;
	}
      }
      break;

    case ON_LINE:
      LOG_NAVI = 1330;
      target_velocity = 50;

      //REF YAW RATE GEN-------------------------------------------------------------
      min_omega = MINUS_RAD_22P5_DEG;
      ref_omega = 0;
      max_omega = RAD_22P5_DEG;
      //LINE TRACE-------------------------------------------------------------------------------
      target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);

      if(odo > ref_odo){
	LOG_NAVI = 1331;
	ZONE    = CORRECT_5TH_ST_ZONE;
	ref_odo = odo + 300;
	gAve_yaw_angle_500->init();
      }
      break;
    default:
      break;
    }
    break;
      

    /** LEFT 2019 ***********************************************************************/
  case CORRECT_5TH_ST_ZONE:
    LOG_NAVI = 1340;
    target_velocity = 50;

    min_omega = MINUS_RAD_15_DEG;
    ref_omega = 0;
    max_omega = RAD_15_DEG;

    //LINE TRACE-------------------------------------------------------------------------------
    target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);
    //--------------------------------------------------------------------------------LINE TRACE
    
    //CORRECT X YAW ANGLE-------------------------------------------------------------
    if( (line_val > 40) && (line_val < 60) ){
      X_POS  = STRAIGT_05[0];
      ave_yaw_angle = gAve_yaw_angle_500->average_500(yaw_angle);
      FL_LOG        = ave_yaw_angle;
    }

    if (odo > ref_odo){
      //CORRECT YAW ANGLE-------------------------------------------------------------
      YAW_ANGLE_OFFSET = RAD_90_DEG - ave_yaw_angle;
      gAve_yaw_angle_500->init();
      ZONE = LAST_STRAIGHT_ZONE;
    }
    break;
    
    /** LEFT 2019 ***********************************************************************/
  case LAST_STRAIGHT_ZONE:
    LOG_NAVI = 1350;
    target_velocity = FIFTH_STRAIGHT_VELOCITY_VAL;
    //    target_omega = omega_frm_vector(STRAIGT_05[2],STRAIGT_05[3], x, y, yaw_angle, velocity);
    target_omega = omega_frm_angle(RAD_90_DEG, yaw_angle);
    if(pre_50mm_y > STRAIGT_05[3]){
      ZONE      = TURN_TO_BLOCK;
      lost_line      = false;
      det_line       = false;
      det_left_edge  = false;
      det_right_edge = false;
      ref_odo        = odo + 200;
    }
    break;



    /** LEFT 2019 ***********************************************************************/
  case TURN_TO_BLOCK:
    LOG_NAVI = 2150;
    target_velocity = 100;
    if(yaw_angle > RAD_91_DEG){
      target_omega = RAD_5_DEG;
    }else{
      target_omega = 0.0;
    }

    if(odo > ref_odo){
      if(line_val > 40){
	ZONE = APPROACH_TO_BLOCK_ZONE;
	ref_odo = odo + 100;
      }
    }

    break;

    /** LEFT 2019 ***********************************************************************/
  case APPROACH_TO_BLOCK_ZONE:
    LOG_NAVI = 2160;

    target_velocity = 100;
    
    //REF YAW RATE GEN-------------------------------------------------------------
    min_omega = MINUS_RAD_22P5_DEG;
    ref_omega = 0;
    max_omega = RAD_22P5_DEG;
    //-------------------------------------------------------------REF YAW RATE GEN

    //LINE TRACE-------------------------------------------------------------------------------
    target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);
    //--------------------------------------------------------------------------------LINE TRACE
    if(x < 1700){ //parameterize lator
      ZONE = BLOCK_ZONE;
      gAve_yaw_angle_500->init();
    }


    break;

/** LEFT 2019 ***********************************************************************/
  case BLOCK_ZONE:
    LOG_NAVI = 2160;

    target_velocity = 50;
    //REF YAW RATE GEN-------------------------------------------------------------
    min_omega = MINUS_RAD_22P5_DEG;
    ref_omega = 0;
    max_omega = RAD_22P5_DEG;

   //-------------------------------------------------------------REF YAW RATE GEN

    //LINE TRACE-------------------------------------------------------------------------------
    target_omega = Navi_Line_Trace->line_trace_omega(line_val, ref_omega, max_omega, min_omega);

    
    //CORRECT Y & YAW ANGLE-------------------------------------------------------------
    if( (line_val > 40) && (line_val < 60) ){
      Y_POS         = 350;
      ave_yaw_angle =gAve_yaw_angle_500->average_500(yaw_angle);
    }
    if (x < 1750){ // modify lator
      	//CORRECT Y YAW ANGLE-------------------------------------------------------------
      YAW_ANGLE_OFFSET = RAD_180_DEG - ave_yaw_angle;
      target_velocity  = 0;
      target_omega     = 0.0;
      ZONE = ENTER_1ST_CORNER_ZONE;
      BLOCK_MODE = true;
    }
    break;

/** LEFT 2019 ***********************************************************************/
  case GARAGE_ZONE:
    LOG_NAVI = 1160;
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



void Navi::block(int line_val, int odo, int velocity, float yaw_angle, int x, int y, bool green_flag){
  int ref_odo;
  float ref_angle;
  switch(BLOCK_MOTION){
  case RX_COMMAND:
    LOG_NAVI = 3000;

    target_velocity = 0;
    target_omega    = 0.0;
    lost_line       = false;
    det_line        = false;
    det_left_edge   = false;
    det_right_edge  = false;

    ref_odo = odo + 350;
    ref_angle = yaw_angle + RAD_90_DEG;
    break;

  case FORWARD:
    LOG_NAVI = 3010;
    if(odo > ref_odo){
      target_velocity = 100;
      target_omega    = 0.0;
      BLOCK_MOTION = RX_COMMAND;
    }else{
      target_velocity = 100;
      target_omega    = omega_frm_angle(ref_angle, yaw_angle);
    }
    break;

  case REVERSE:
    LOG_NAVI = 3020;
    if(odo < ref_odo){
      target_velocity = 100;
      target_omega    = 0.0;
      BLOCK_MOTION = RX_COMMAND;
    }else{
      target_velocity = -100;
      target_omega    = 0.0;
    }
    break;

  case LEFT_LINE_DET:
    LOG_NAVI = 3100;
    if(yaw_angle < ref_angle){
      target_velocity = 0;
      target_omega    = RAD_22P5_DEG;
    }else{
      BLOCK_MOTION = RIGHT_LINE_DET;
    }

    if(line_val > 60){
      det_line = true;
    }
    if(det_line){
      LOG_NAVI = 3110;
      if(line_val < 20){
	lost_line = true;
	det_left_edge = true;
      }
    }

    if(det_left_edge){
      LOG_NAVI = 3120;
      if(line_val > 50){
	BLOCK_MOTION = RX_COMMAND;
      }else{
	target_velocity = 0;
	target_omega    = MINUS_RAD_22P5_DEG;
      }
    }
    break;

  case RIGHT_LINE_DET:
    LOG_NAVI = 3200;
    if(yaw_angle > ref_angle){
      target_velocity = 0;
      target_omega    = MINUS_RAD_22P5_DEG;
    }else{
      BLOCK_MOTION = LEFT_LINE_DET;
    }

    if(line_val > 60){
      det_line = true;
      det_left_edge = true;
      BLOCK_MOTION = RX_COMMAND;
    }

    break;

  case LEFT_90_TURN:
    target_velocity = 0;
    target_omega    = omega_frm_angle(ref_angle, yaw_angle);
    if(yaw_angle > (ref_angle - RAD_1_DEG)){

    }
    break;

  case RIGHT_90_TURN:

    break;

  case LEFT_45_TURN:

    break;

  case RIGHT_45_TURN:

    break;

  case FORWARD_TO_CIRCLE:

    break;

  case LEFT_180_TURN:

    break;
  case GOAL:


  default:
    break;
  }
}

