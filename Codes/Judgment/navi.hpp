/******************************************************************************
 *  navi.h (for LEGO Mindstorms EV3)
 *  Created on: 2018/04/21
 *  Implementation of the Class navi
 *  Author: Kaoru Ota
 *****************************************************************************/

#ifndef EV3_APP_NAVI_H_
#define EV3_APP_NAVI_H_
#include "util.hpp"
#include "line_tracer.hpp"
#include "parameter.h"

using namespace std;

class Navi {
public:
  explicit Navi();//コンストラクタ
  void init();
  float omega_frm_vector(float target_x, float target_y, float current_x, float current_y, float yaw_angle, int velocity);
  float omega_frm_circle(float circle_x, float circle_y, float circle_r, float current_x, float current_y, float yaw_angle, int velocity);
  float omega_frm_angle(float target_angle, float yaw_angle);


  //  void run(int line_val,int odo, int velocity, float yaw_angle, float ave_yaw_angle, int x, int y, int pre_50mm_x, int pre_50mm_y);
  void run(int line_val,int odo, int velocity, float yaw_angle, int x, int y, int pre_50mm_x, int pre_50mm_y);
  void block(int line_val,int odo, int velocity, float yaw_angle, int x, int y);
  
  Average_500_Data *gAve_yaw_angle_500 = new Average_500_Data(); //20181108
  Average_500_Data *gAve_x_500 = new Average_500_Data();
  Average_500_Data *gAve_y_500 = new Average_500_Data();

  int   target_velocity;
  float target_omega;
  float min_omega;
  float ref_omega;
  float max_omega;

  float ave_x;
  float ave_y;
  float ave_yaw_angle;

  bool  lost_line;
  bool  det_line;
  bool  det_left_edge;
  bool  det_right_edge;



private:
  Line_Trace *Navi_Line_Trace = new Line_Trace();

  enum Zone{
    START_ZONE,
    START_BACK,
    FIRST_STRAIGHT_ZONE,
    ENTER_1ST_CORNER_ZONE,
    FIRST_CORNER_ZONE,//CORRECT X YAW ANGLE-------------------------------------------------------------
    SECOND_STRAIGHT_ZONE,
    ENTER_2ND_CORNER_ZONE,
    SECOND_CORNER_ZONE,
    THIRD_STRAIGHT_ZONE,
    FIND_3RD_CORNER,
    GATE_01,

    THIRD_CORNER_ZONE,
    S_CORNER_ZONE,

    CORRECT_4TH_ST_ZONE,
    FOURTH_STRAIGHT_ZONE,
    FOURTH_CORNER_ZONE,
    ENTER_5TH_CORNER_ZONE,
    FIFTH_CORNER_ZONE,
    SIXTH_CORNER_ZONE,
    SEVENTH_CORNER_ZONE,
    EIGHTH_CORNER_ZONE,
    NINTH_CORNER_ZONE,
    TENTH_CORNER_ZONE,
    FIFTH_STRAIGHT_ZONE,
    APPROACH_TO_BLOCK_ZONE,
    BLOCK_ZONE,
    GARAGE_ZONE,
    LOST
  };

  enum Find_Line{
		 LEFT_SEARCH,
		 RIGHT_SEARCH,
		 FORWARD_SEARCH,
		 BACKWARD_SEARCH,
		 ON_LINE,
		 DET_LINE,
		 LOST_LINE
  };

  enum Block_Motion{
    RX_COMMAND,
    FORWARD,
    REVERSE,
    LEFT_LINE_DET,
    RIGHT_LINE_DET,
    LEFT_90_TURN,
    RIGHT_90_TURN,
    LEFT_45_TURN,
    RIGHT_45_TURN,
    FORWARD_TO_CIRCLE,
    LEFT_180_TURN,
    GOAL
  };



  Zone         ZONE;
  Find_Line    FIND_LINE;
  Block_Motion BLOCK_MOTION;

};

#endif  // EV3_APP_NAVI_H_
