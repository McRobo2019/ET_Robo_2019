/******************************************************************************
 *  navi.h (for LEGO Mindstorms EV3)
 *  Created on: 2018/04/21
 *  Implementation of the Class navi
 *  Author: Kaoru Ota
 *****************************************************************************/

#ifndef EV3_APP_NAVI_H_
#define EV3_APP_NAVI_H_
#include "util.hpp"
#include "parameter.h"

using namespace std;

class Navi {
public:
  explicit Navi();//コンストラクタ
  void init();
  void run(int odo, int velocity, float yaw_angle, float ave_yaw_angle, int x, int y, int pre_50mm_x, int pre_50mm_y);

  Average_500_Data *gAve_yaw_angle_500 = new Average_500_Data(); //20181108

  float ave_yaw_angle_500; //20181108
  int   target_velocity;
  float min_omega;
  float ref_omega;
  float max_omega;
  bool  lost_line;

private:
  enum Zone{
    START_ZONE,
    START_BACK,
    FIRST_STRAIGHT_ZONE,
    ENTER_1ST_CORNER_ZONE,
    FIRST_CORNER_ZONE,
    SECOND_STRAIGHT_ZONE,
    ENTER_2ND_CORNER_ZONE,
    SECOND_CORNER_ZONE,
    THIRD_STRAIGHT_ZONE,
    THIRD_CORNER_ZONE,
    S_CORNER_ZONE,
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
    FIRST_GRAY_ZONE,
    LUG_ZONE,
    BACK_LUG_ZONE,
    SECOND_GRAY_ZONE,
    SEESAW_ZONE,
    GARAGE_ZONE,
    LOST
  };

  Zone         ZONE;

};

#endif  // EV3_APP_NAVI_H_
