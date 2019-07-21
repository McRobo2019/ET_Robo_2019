#include "recognition.hpp"
#include "math.h"
#include "Clock.h"


//using namespace ev3api;
using ev3api::Clock;

Clock* Sys_Clock;

/**
 * コンストラクタ
 * @param colorSensor カラーセンサ
 */
Recognition::Recognition(const ev3api::ColorSensor& colorSensor,
		 ev3api::Motor& leftWheel,
		 ev3api::Motor& rightWheel,
		 ev3api::GyroSensor& gyro,
		 ev3api::SonarSensor& sonar)
  : 
    mColorSensor(colorSensor),
    mLeftWheel(leftWheel),
    mRightWheel(rightWheel),
    mGyro(gyro),
    mSonar(sonar)
{
}

void Recognition::init(){
  Sys_Clock    = new Clock();
  mGyro.reset();

  //  real_wheel = WHEEL_R * cos(RAD_6_DEG);
  //real_wheel = WHEEL_R * 1.05; /*180630 temporary value*/
  real_wheel = WHEEL_R;
  xvalue    = X_POS_OFFSET;
  yvalue    = Y_POS_OFFSET;

  CORRECT_MODE = ST_1ST;
  pre_sonar_dis = 0;


  pre_50mm_x = 0.0;
  pre_50mm_y = 0.0;

  ave_x      = X_POS_OFFSET; //average_x 20181008
  ave_y      = Y_POS_OFFSET; //average_y 20181008

  ave_vel_x = 0;    //20181008
  ave_vel_y = 0;    //20181008


  odo             = 0.0;
  velocity        = 0.0;
  ave_velo        = 0.0; //20181008
  ave_accel       = 0.0; //20181008
  pre_velo_0p5sec = 0.0;//20181008

  encR      = 0;
  encL      = 0;

  pre_encR  = 0;
  pre_encL  = 0;

  wheel_rotational_speed = 0;
  ave_wheel_rot_speed    = 0;
  abs_ave_wheel_speed    = 0;
  wheel_load             = 0;

  yawrate   = 0;
  abs_angle = 0;
  ave_angle = 0;

  robo_stop       = 1;
  robo_forward    = 0;
  robo_back       = 0;
  robo_turn_left  = 0;
  robo_turn_right = 0;

  gAve_angle_dat->init(0.0);          

  gAve_x_dat->init(X_POS_OFFSET);          
  gAve_y_dat->init(Y_POS_OFFSET);

  gAve_vel_x_dat->init(0.0);
  gAve_vel_y_dat->init(0.0);

  gAve_velo_dat->init(0.0);
  gAve_accel_dat->init(0.0);

  gAve_wheel_rot_dat->init(0.0);
  gAve_wheel_load_dat->init(0.0);

  gAve_angle_500_dat->init();

}

void Recognition::run( ) {

  SYS_CLK = Sys_Clock->now();

  det_line_rgb();
  //wheel_odometry(dT_4ms);
  wheel_odometry(dT_10ms);
  //average_dat(dT_4ms);
  average_dat(dT_10ms);
  det_Movement(); //20180501
}


void Recognition::det_line_rgb(){

  rgb_raw_t rgb_val;
  float adj_brightness;

  mColorSensor.getRawColor(rgb_val);


  color_r = (int)rgb_val.r;
  color_g = (int)rgb_val.g;
  color_b = (int)rgb_val.b;


  adj_brightness = rgb_val.b - COLOR_SENSOR_OFFSET;
  adj_brightness = adj_brightness/COLOR_SENSOR_GAIN;



  if(adj_brightness < 0){
    adj_brightness = 0;
  }
  else if(adj_brightness > 100){
    adj_brightness = 100;
  }
  linevalue = 100-adj_brightness;

  if((rgb_val.r < 10) && (rgb_val.g > rgb_val.b)){
    green_flag = 1;
  }else{
    green_flag = 0;
  }

}

void Recognition::wheel_odometry(float dT) {

  static float odo_prev;

  static float velocity_input;
  static float velocity_prev;

  static float left_wheel_velo_input;
  static float left_wheel_velo_prev;

  static float right_wheel_velo_input;
  static float right_wheel_velo_prev;

  //LPF 10[rad/s]/////////////////////////////////////
  static float Alpfd = 0.9391; // LPF
  static float Blpfd = 1; // LPF
  static float Clpfd = 0.0609; // LPF
  static float Dlpfd = 0; // LPF
  //////////////////////////////////////////
  static float old_rel_angle;     //過去のYaw角[rad]

  int   WheelAngRdeg = mRightWheel.getCount();  //右モータ回転角度[deg]
  int   WheelAngLdeg = mLeftWheel.getCount();   //右モータ回転角度[deg]

  encR =  WheelAngRdeg;
  encL =  WheelAngLdeg;
  
  odo            = ((float)WheelAngLdeg + (float)WheelAngRdeg)/2.0 * RAD_1_DEG * real_wheel; //[mm]

  velocity_input = (odo - odo_prev)/dT;
  velocity       = Clpfd * velocity_prev + Dlpfd * velocity_input;
  velocity_prev  = Alpfd * velocity_prev + Blpfd * velocity_input;
  
  left_wheel_velo_input = ((encL - pre_encL)*RAD_1_DEG*real_wheel)/dT;
  left_wheel_velocity   = Clpfd * left_wheel_velo_prev + Dlpfd * left_wheel_velo_input;
  left_wheel_velo_prev  = Alpfd * left_wheel_velo_prev + Blpfd * left_wheel_velo_input;
  
  right_wheel_velo_input = ((encR - pre_encR)*RAD_1_DEG*real_wheel)/dT;
  right_wheel_velocity   = Clpfd * right_wheel_velo_prev + Dlpfd * right_wheel_velo_input;
  right_wheel_velo_prev  = Alpfd * right_wheel_velo_prev + Blpfd * right_wheel_velo_input;

  pre_encL = encL;
  pre_encR = encR;


  omega = (right_wheel_velocity - left_wheel_velocity)/RoboTread;

  relative_angle =  ((float)WheelAngRdeg - (float)WheelAngLdeg) * RAD_1_DEG * real_wheel / RoboTread; //ロボのYaw角[rad]
  abs_angle      = relative_angle + correction_angle;

  xvalue = xvalue+(odo-odo_prev)*cos(abs_angle);
  yvalue = yvalue+(odo-odo_prev)*sin(abs_angle);

  pre_50mm_x = xvalue + 50.0*cos(abs_angle);//20180512 kota
  pre_50mm_y = yvalue + 50.0*sin(abs_angle);


  yawrate  =(relative_angle-old_rel_angle)/dT;           //ロボのYawレート[rad/s]

  old_rel_angle=relative_angle;         //過去のYaw角[rad]
  odo_prev = odo;
}

void Recognition::average_dat(float dT) {
  ave_angle           = gAve_angle_dat->average_125(abs_angle);
  ave_velo            = gAve_velo_dat->average_125(velocity);
  ave_wheel_rot_speed = gAve_wheel_rot_dat->average_125(wheel_rotational_speed);
  ave_wheel_load      = gAve_wheel_load_dat->average_125(wheel_load);
  ave_x               = gAve_x_dat->average_125(xvalue);
  ave_y               = gAve_y_dat->average_125(yvalue);

  dif_angle_ave_dat    = ave_angle    - old_angle_ave_dat;
  old_angle_ave_dat    = ave_angle;
 
  dif_velocity_ave_dat = ave_velo - old_velocity_ave_dat;
  old_velocity_ave_dat = ave_velo;
 
  dif_x_ave_dat        = ave_x - old_x_ave_dat;
  old_x_ave_dat        = ave_x;
 
  dif_y_ave_dat        = ave_y - old_y_ave_dat;
  old_y_ave_dat        = ave_y;
  
  ave_accel = dif_velocity_ave_dat/dT;
 
  ave_vel_x = dif_x_ave_dat/dT;
  ave_vel_y = dif_y_ave_dat/dT;
 
  pre_velo_0p5sec  =  ave_velo + (ave_accel * 0.5);
  
}

void Recognition::correct_odometry( ) {

  /* 2018 LEFT */
  int   dif_sonar_dis;
  float correct_x;
  float correct_y;

  static float x_min;
  static float x_max;
  static float ref_y;


  switch(CORRECT_MODE){
  case ST_1ST:
    if((xvalue > (FIRST_STRAIGHT_AREA[2] - 300))&&(robo_forward == 1)){
      correction_angle = 0.0 - ave_angle;
      yvalue = 160.0;
      CORRECT_MODE = ST_2ND;
    }
    
    break;
    
  case ST_2ND:
    if((yvalue > (SECOND_STRAIGHT_AREA[3] - 300))&&(robo_forward == 1)){
      correction_angle = RAD_90_DEG - ave_angle;
      xvalue = 3770.0;
      CORRECT_MODE = ST_3RD;
    }

    x_max = 0;
    break;

  case ST_3RD:
    if((xvalue > (THIRD_STRAIGHT_AREA[0] + 200))&&(robo_forward == 1)){
      correction_angle = 0.0 - ave_angle;
      yvalue = 3270;
      x_min  = xvalue;
  //	 CORRECT_MODE = DONE;
      //      CORRECT_MODE = CN_4TH;
      CORRECT_MODE = ST_4TH;
    }
    
    if(xvalue > x_max){
      x_max = xvalue;
    }

    break;

  case ST_4TH:

    if(xvalue > x_max){
      x_max = xvalue;
    }

    if( yvalue < 2000 ){
      correct_x = 5020 - x_max;
      xvalue = xvalue + correct_x;
      CORRECT_MODE = CN_4TH;
    }

    break;



  case CN_4TH:

    if(xvalue < x_min){
      x_min = xvalue;
      ref_y = yvalue;
    }

    if( yvalue < 280 ){
      correct_x = 3580 - x_min;
      xvalue = xvalue + correct_x;

      correct_y = 450 - ref_y;
      yvalue = yvalue + correct_y;
      CORRECT_MODE = LUG;
      pre_sonar_dis = sonarDistance;
    }

    /*
    if((yvalue < FIRST_GRAY_AREA[3]) && (ave_angle > MINUS_RAD_5_DEG)){

    if( (xvalue > 3770 )&&( xvalue < 3970) ){
    xvalue = 3870;
    yvalue = 160;
    }

    CORRECT_MODE = LUG;
    pre_sonar_dis = sonarDistance;

    }*/



    break;

  case LUG:
    dif_sonar_dis = sonarDistance - pre_sonar_dis;
   
    if( (dif_sonar_dis > 100) && (pre_sonar_dis < 20) && (xvalue > 4100) ){
      xvalue       = BACK_LUG_AREA[0] - (pre_sonar_dis * 10);
      CORRECT_MODE = DONE;
    }

    pre_sonar_dis = sonarDistance;

    break;
    
  case DONE:

    break;

  default:
    break;
    }
}



//!!!! Not Done Any Verification Yet 20180501 !!!!
/****************************************************************************************/
//2018 0501 Kaoru Ota
//Move DET_MOVEMENT from :wheel_odometry(float dT)
//!!!! Not Done Any Verification Yet 20180501 !!!!
/****************************************************************************************/
void Recognition::det_Movement( ) {

  if (dif_angle_ave_dat > -0.001 && dif_angle_ave_dat < 0.001){

    if(ave_velo > 0){
      robo_stop       = 0;
      robo_forward    = 1;
      robo_back       = 0;
      robo_turn_left  = 0;
      robo_turn_right = 0;
    }else if (ave_velo < 0){
      robo_stop       = 0;
      robo_forward    = 0;
      robo_back       = 1;
      robo_turn_left  = 0;
      robo_turn_right = 0;
    }else{
      robo_stop       = 1;
      robo_forward    = 0;
      robo_back       = 0;
      robo_turn_left  = 0;
      robo_turn_right = 0;
    }

  }else if(dif_angle_ave_dat >= 0.001){
    robo_stop       = 0;
    robo_forward    = 0;
    robo_back       = 0;
    robo_turn_left  = 1;
    robo_turn_right = 0;
  }else if(dif_angle_ave_dat <= -0.001){
    robo_stop       = 0;
    robo_forward    = 0;
    robo_back       = 0;
    robo_turn_left  = 0;
    robo_turn_right = 1;
  }else {
    robo_stop       = 0;
    robo_forward    = 0;
    robo_back       = 0;
    robo_turn_left  = 0;
    robo_turn_right = 0;
  }
}


void Recognition::setSonarDistance(void) {

  if(sonar_counter%50 == 0){
    sonarDistance = mSonar.getDistance();
  }
  sonar_counter++;
  if(sonar_counter>10000) sonar_counter = 0;
 
}



