/*
#ifdef __cplusplus
extern "C" {
#endif

#define dT_100ms             0.1 //タスク周期[s]

#ifdef __cplusplus
}
#endif
*/

//Parameter of System
extern int SYS_CLK;

//Parameter of Robo
extern int ARM_ANGLE_LT;
extern int TAIL_ANGLE_BALANCE_START;
extern int TAIL_ANGLE_LAUNCH;
extern int TAIL_ANGLE_RUN;     /* バランス走行時の角度[度] */
extern int TAIL_ON_ANGLE; /* 完全停止時の角度[度] */
extern int TAIL_ANGLE_LUG; /* 3点移動時の角度[度] */
extern int TAIL_ANGLE_GARAGE; /* 完全停止時の角度[度] */

extern float WHEEL_R;       //radius of wheel[mm]
extern float RoboTread;      //トレッド長さ[mm]
extern float HALF_TREAD;

extern float MAX_VELOCITY;


//Parameter of time length unit
extern float dT_4ms;
extern float dT_10ms;

extern float PAI;

extern float RAD_315_DEG;
extern float RAD_1_DEG; //deg@1rad 
extern float RAD_5_DEG; //
extern float RAD_6_DEG;
extern float RAD_15_DEG; //deg@1rad 
extern float RAD_22P5_DEG; //
extern float RAD_30_DEG; //
extern float RAD_45_DEG; //
extern float RAD_89_DEG;
extern float RAD_88p5_DEG;
extern float RAD_87_DEG;
extern float RAD_90_DEG;
extern float RAD_120_DEG;
extern float RAD_135_DEG;
extern float RAD_150_DEG;
extern float RAD_180_DEG;
extern float RAD_225_DEG;
extern float RAD_270_DEG;
extern float RAD_315_DEG;
extern float RAD_345_DEG;
extern float RAD_360_DEG;
extern float RAD_450_DEG;

extern float MINUS_RAD_5_DEG;    //
extern float MINUS_RAD_15_DEG;   //
extern float MINUS_RAD_22P5_DEG; //
extern float MINUS_RAD_30_DEG;   //
extern float MINUS_RAD_45_DEG;   //
extern float MINUS_RAD_60_DEG;   //
extern float MINUS_RAD_90_DEG;   //
extern float MINUS_RAD_135_DEG;   //
extern float MINUS_RAD_145_DEG;  //
extern float MINUS_RAD_180_DEG;  //
extern float MINUS_RAD_225_DEG;  //
extern float MINUS_RAD_270_DEG;  //

//Odometry
extern int   ODO;
extern int   X_POS;
extern int   Y_POS;
extern float X_UNIT_VEC;
extern float Y_UNIT_VEC;
extern float YAW_ANGLE;

extern int   D_ODO;
extern int   D_X_POS;
extern int   D_Y_POS;
extern float D_YAW_ANGLE;

extern int   PRE_X_POS;
extern int   PRE_Y_POS;


//extern int   X_POS_INIT;
//extern int   Y_POS_INIT;
extern float YAW_ANGLE_INIT;

extern float YAW_ANGLE_OFFSET;

extern float YAW_LIMIT;
extern float YAW_STEP;

//Parameter of Motor CTL
extern float MOTOR_CTL_TS;
extern float MOTOR_CTL_KI;
extern float MOTOR_CTL_KP;


//LUG

extern int   MAX_LUG_FORWARD;
extern float LUG_1st_STOP_X;
extern float LUG_2nd_STOP_X;
extern float LUG_3rd_STOP_X;

extern float LUG_YAW_GAIN;
extern int   TAIL_STD_LINE_DET;

extern int   CALIB_LINE_100_MAX_THRS;
extern int   CALIB_LINE_50_MAX_THRS;
extern int   CALIB_LINE_50_MIN_THRS;
extern int   CALIB_LINE_0_MIN_THRS;

//Color Sensor Parameter
extern int   COLOR_SENSOR_OFFSET;
extern float COLOR_SENSOR_GAIN;

//Parameter of Garage
extern float GARAGE_X_POS;


extern float ACCEL_GAIN;
extern float DECEL_GAIN;

//Parameter of Area
extern int START_VELOCITY_VAL;
extern int FIRST_STRAIGHT_VELOCITY_VAL;
extern int ENTER_1ST_CORNER_VELOCITY_VAL;
extern int FIRST_CORNER_VELOCITY_VAL;
extern int SECOND_STRAIGHT_VELOCITY_VAL;
extern int ENTER_2ND_CORNER_VELOCITY_VAL;
extern int SECOND_CORNER_VELOCITY_VAL;
extern int THIRD_STRAIGHT_VELOCITY_VAL;
extern int THIRD_CORNER_VELOCITY_VAL;
extern int S_CORNER_VELOCITY_VAL;
extern int FOURTH_STRAIGHT_VELOCITY_VAL;
extern int FOURTH_CORNER_VELOCITY_VAL;
extern int ENTER_5TH_CORNER_VELOCITY_VAL;
extern int FIFTH_CORNER_VELOCITY_VAL;
extern int SIXTH_CORNER_VELOCITY_VAL;
extern int SEVENTH_CORNER_VELOCITY_VAL;
extern int EIGHTH_CORNER_VELOCITY_VAL;
extern int NINTH_CORNER_VELOCITY_VAL;
extern int TENTH_CORNER_VELOCITY_VAL;


extern int FIRST_GRAY_VELOCITY_VAL;
extern int LUG_VELOCITY_VAL;
extern int BACK_LUG_VELOCITY_VAL;
extern int SECOND_GRAY_VELOCITY_VAL;
extern int SEESAW_VELOCITY_VAL;
extern int GARAGE_VELOCITY_VAL;
extern int GOAL_VAL;

extern int START_AREA[4];
extern int FIRST_STRAIGHT_AREA[4];
extern int ENTER_1ST_CORNER_AREA[4];
extern int FIRST_CORNER_AREA[4];
extern int SECOND_STRAIGHT_AREA[4];
extern int ENTER_2ND_CORNER_AREA[4];
extern int SECOND_CORNER_AREA[4];
extern int THIRD_STRAIGHT_AREA[4];
extern int THIRD_CORNER_AREA[4];
extern int S_CORNER_AREA[4];
extern int FOURTH_STRAIGHT_AREA[4];
extern int FOURTH_CORNER_AREA[4];
extern int ENTER_5TH_CORNER_AREA[4];
extern int FIFTH_CORNER_AREA[4];
extern int SIXTH_CORNER_AREA[4];
extern int SEVENTH_CORNER_AREA[4];
extern int EIGHTH_CORNER_AREA[4];
extern int NINTH_CORNER_AREA[4];
extern int TENTH_CORNER_AREA[4];

extern int FIFTH_STRAIGHT_AREA[4];

extern int FIRST_GRAY_AREA[4];
extern int LUG_AREA[4];
extern int BACK_LUG_AREA[4];
extern int SECOND_GRAY_AREA[4];
extern int SEESAW_AREA[4];
extern int GARAGE_AREA[4];

extern int STRAIGT_01[4];
extern int CIRCLE_01[3];
extern int STRAIGT_02[4];
extern int CIRCLE_02[3];
extern int STRAIGT_03[4];
extern int CIRCLE_03[3];
extern int STRAIGT_04[4];
extern int CIRCLE_04[3];
extern int STRAIGT_05[4];
extern int CIRCLE_05[3];
extern int CIRCLE_06[3];
extern int CIRCLE_07[3];
extern int CIRCLE_08[3];
extern int CIRCLE_09[3];
extern int CIRCLE_10[3];

extern int CIRCLE_01_LENGTH;
extern float CIRCLE_01_ANGLE;

extern int CIRCLE_02_LENGTH;
extern float CIRCLE_02_ANGLE;

extern int CIRCLE_03_LENGTH;
extern float CIRCLE_03_ANGLE;

extern int CIRCLE_04_LENGTH;
extern float CIRCLE_04_ANGLE;

extern int CIRCLE_05_LENGTH;
extern float CIRCLE_05_ANGLE;

extern int LOG_NAVI;
