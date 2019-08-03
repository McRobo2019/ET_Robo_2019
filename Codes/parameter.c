/**** SEE           ****/
/**** ADJ_PARAMETER ****/


//Parameter of System
int SYS_CLK = 0;


//Parameter of Robo
int ARM_ANGLE_LT       = 30;
int TAIL_ANGLE_BALANCE_START  = 100;
int TAIL_ANGLE_LAUNCH         = 105;


int TAIL_ANGLE_RUN      =  3; /* バランス走行時の角度[度] */
int TAIL_ON_ANGLE       = 85; /* 完全停止時の角度[度]     */
//int TAIL_ANGLE_LUG      = 75; /* 3点移動時の角度[度]      */
//int TAIL_ANGLE_LUG      = 70; /* 3点移動時の角度[度]      */
//int TAIL_ANGLE_LUG      = 65; /* 3点移動時の角度[度]      */
//int TAIL_ANGLE_LUG      = 68; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_LUG      = 67; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_GARAGE   = 94; /* 完全停止時の角度[度]     */

//float WHEEL_R       = 49.75; //Wheel radius 2018
float WHEEL_R       = 50.0; //Wheel radius 20190527 refer from 2018 cs model
//int   RoboTread     = 155; //トレッド長さ[mm]
float   RoboTread     = 147.6; //トレッド長さ[mm] 20190527, ota, refer from 2018 cs mode 
float   HALF_TREAD    = 73.8; 

float MAX_VELOCITY      = 400; /**** ADJ_PARAMETER ****/
//float MAX_VELOCITY      = 450; /**** ADJ_PARAMETER ****/
//float MAX_VELOCITY      = 500; /**** ADJ_PARAMETER ****/

//Parameter of time length unit
float dT_4ms   = 0.004;
float dT_10ms   = 0.010;

//float PAI         =  3.1472;
float PAI         =  3.141592;

float RAD_1_DEG    = 0.0175; //deg@1rad 
float RAD_5_DEG    = 0.0873; //
float RAD_6_DEG    = 0.1047; //
float RAD_15_DEG   = 0.2618; //
float RAD_22P5_DEG = 0.3927; //
float RAD_30_DEG   = 0.5236; //
float RAD_45_DEG   = 0.7854; //
float RAD_60_DEG   = 1.0472; //
float RAD_89_DEG   = 1.5533; //
float RAD_88p5_DEG = 1.5446; //
float RAD_87_DEG   = 1.5184; //
float RAD_90_DEG   = 1.5708; //
float RAD_120_DEG  = 2.0944; //
float RAD_135_DEG  = 2.3562; //
float RAD_150_DEG  = 2.6180; //
float RAD_180_DEG  = 3.1472; //
float RAD_225_DEG  = 3.9270; //
float RAD_270_DEG  = 4.7124; //
float RAD_315_DEG  = 5.4978; //
float RAD_345_DEG  = 6.0214; //
float RAD_360_DEG  = 6.2832; //
float RAD_450_DEG  = 7.8540;

float MINUS_RAD_5_DEG    = -0.0873; //
float MINUS_RAD_15_DEG   = -0.2618; //
float MINUS_RAD_22P5_DEG = -0.3927; //
float MINUS_RAD_30_DEG   = -0.5236; //
float MINUS_RAD_45_DEG   = -0.7854; //
float MINUS_RAD_60_DEG   = -1.0472; //
float MINUS_RAD_90_DEG   = -1.5708; //
float MINUS_RAD_135_DEG  = -2.3562; //
float MINUS_RAD_145_DEG  = -2.5307; //
float MINUS_RAD_180_DEG  = -3.1472; //
float MINUS_RAD_225_DEG  = -3.9270; //
float MINUS_RAD_270_DEG  = -4.7124;


//Odometry
int   LINE_VAL   = 0;
float ODO        = 0.0;
float X_POS      = 480.0;
float Y_POS      = 165.0;
float X_UNIT_VEC = 0.0;
float Y_UNIT_VEC = 0.0;
float YAW_ANGLE  = 0.0;

float D_ODO       = 0.0;
float D_X_POS     = 0.0;
float D_Y_POS     = 0.0;
float D_YAW_ANGLE = 0.0;

float PRE_X_POS   = 0.0;
float PRE_Y_POS   = 0.0;

//int X_POS_INIT       = 480;
//int Y_POS_INIT       = 165;
float YAW_ANGLE_INIT   = 0.0;
float YAW_ANGLE_OFFSET = 0.0;

float YAW_LIMIT    = 0.393;   //PI/8 see anago synario
float YAW_STEP     = 0.00786; //YAW_LIMIT/50 see anago synario

//Parameter of Motor CTL 20190719 ota add
float MOTOR_CTL_TS = 0.01;

//float MOTOR_CTL_KI = 0.47782874617737003;
//float MOTOR_CTL_KI = 1.911314984709480; // A
//float MOTOR_CTL_KI = 2.645418663957758; // B
//float MOTOR_CTL_KI = 3.397893306150187; // C
float MOTOR_CTL_KI = 0.9; // test 

//float MOTOR_CTL_KP = 0.0382262996941896;
//float MOTOR_CTL_KP = 0.229357798165138; // A 
//float MOTOR_CTL_KP = 0.296815974096060; // B
//float MOTOR_CTL_KP = 0.356778797145770; // C
float MOTOR_CTL_KP = 0.1; //test


//--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--
//--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--LUG--

int   MAX_LUG_FORWARD        = 30;

//map data
//https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo

float LUG_1st_STOP_X         = 4590; /**** ADJ_PARAMETER ****/
//float LUG_2nd_STOP_X         = 4190; /**** ADJ_PARAMETER ****/
float LUG_2nd_STOP_X         = 4150; /**** ADJ_PARAMETER ****/
float LUG_3rd_STOP_X         = 4590; /**** ADJ_PARAMETER ****/

float LUG_YAW_GAIN           = 2.0;

//Parameter of Garage
//float GARAGE_X_POS          = 4980;
float GARAGE_X_POS          = 4990; /**** ADJ_PARAMETER   case GRAY_GARAGE:****/

//int   TAIL_STD_LINE_DET      = 49; /**** ADJ_PARAMETER ****/
int   TAIL_STD_LINE_DET      = 70;  /**** ADJ_PARAMETER ****/


//Color Sensor Paramter
/*
int   CALIB_LINE_100_MAX_THRS = 20;
int   CALIB_LINE_50_MAX_THRS  = 100;
int   CALIB_LINE_50_MIN_THRS  = 40;
int   CALIB_LINE_0_MIN_THRS   = 150;
*/

int   CALIB_LINE_100_MAX_THRS = 20;
int   CALIB_LINE_50_MAX_THRS  = 100;
int   CALIB_LINE_50_MIN_THRS  = 40;
int   CALIB_LINE_0_MIN_THRS   = 50;

//HONBAN
int   COLOR_SENSOR_OFFSET    = 18;  /**** ADJ_PARAMETER ****/
//int   COLOR_SENSOR_OFFSET    = 10;  /**** ADJ_PARAMETER ****/
//float COLOR_SENSOR_GAIN      = 2.0; /**** ADJ_PARAMETER ****/
float COLOR_SENSOR_GAIN      = 3.0; /**** ADJ_PARAMETER ****/

//YOBI
//int   COLOR_SENSOR_OFFSET    = 10;  /**** ADJ_PARAMETER ****/
//float COLOR_SENSOR_GAIN      = 1.5; /**** ADJ_PARAMETER ****/


//map data
//https://drive.google.com/open?id=1Ih-fZX68pgRuQWVPbCW-5Oj-Bgrn3pgo
//(x0,y0) - (x1,y1)

float ACCEL_GAIN                = 1.0;
float DECEL_GAIN                = 1.0;

int START_VELOCITY_VAL            = 100; //koko
int FIRST_STRAIGHT_VELOCITY_VAL   = 100;
int ENTER_1ST_CORNER_VELOCITY_VAL = 100;
int FIRST_CORNER_VELOCITY_VAL     = 100;
int SECOND_STRAIGHT_VELOCITY_VAL  = 100;
int ENTER_2ND_CORNER_VELOCITY_VAL = 100;
int SECOND_CORNER_VELOCITY_VAL    = 100;
int THIRD_STRAIGHT_VELOCITY_VAL   = 100;
int THIRD_CORNER_VELOCITY_VAL     = 100;
int S_CORNER_VELOCITY_VAL         = 100;
int FOURTH_STRAIGHT_VELOCITY_VAL  = 100;
int GOAL_VAL                     = 100;

int FOURTH_CORNER_VELOCITY_VAL    = 100;

int ENTER_5TH_CORNER_VELOCITY_VAL = 100;
int FIFTH_CORNER_VELOCITY_VAL     = 100;
int SIXTH_CORNER_VELOCITY_VAL     = 100;
int SEVENTH_CORNER_VELOCITY_VAL     = 100;
int EIGHTH_CORNER_VELOCITY_VAL     = 100;
int NINTH_CORNER_VELOCITY_VAL     = 100;
int TENTH_CORNER_VELOCITY_VAL     = 100;

int FIRST_GRAY_VELOCITY_VAL       = 100;
int LUG_VELOCITY_VAL              = 100;
int BACK_LUG_VELOCITY_VAL         = 100;
int SECOND_GRAY_VELOCITY_VAL      = 100;
int SEESAW_VELOCITY_VAL           = 100;
int GARAGE_VELOCITY_VAL           = 100;


int START_AREA[4]            = { 300,    0,  500,  330};
int FIRST_STRAIGHT_AREA[4]   = { 500,    0, 1140,  330};
int ENTER_1ST_CORNER_AREA[4] = { 800,    0, 1140,  605};
int FIRST_CORNER_AREA[4]     = {1140,    0, 1745,  605};
int SECOND_STRAIGHT_AREA[4]  = {1400,  605, 1745,  900};
int ENTER_2ND_CORNER_AREA[4] = {1400,  900, 1745, 1150};
int SECOND_CORNER_AREA[4]    = { 950, 1150, 1745, 1800};
//int THIRD_STRAIGHT_AREA[4]   = {0, 0, 0, 0};
int THIRD_CORNER_AREA[4]     = {500, 1100, 950, 1800};
//int FOURTH_STRAIGHT_AREA[4]  = {0,  0, 0, 0};
int FOURTH_CORNER_AREA[4]    = {500, 420, 950,  1100};
int ENTER_5TH_CORNER_AREA[4] = {   0,    0,    0,    0};
int FIFTH_CORNER_AREA[4]     = {  0,   420, 500,  890};
int THIRD_STRAIGHT_AREA[4]   = {  0,   890, 500, 1100};
int SIXTH_CORNER_AREA[4]     = {  0,  1100, 605, 1800};
int FOURTH_STRAIGHT_AREA[4]  = { 605, 1450, 1625, 1800};
int SEVENTH_CORNER_AREA[4]   = {1625, 1090, 2215, 1800};
int EIGHTH_CORNER_AREA[4]    = {1625,  840, 2215, 1090};
int NINTH_CORNER_AREA[4]     = {1625,    0, 2075,  840};
int TENTH_CORNER_AREA[4]     = {2075,    0, 2700,  540};
int FIFTH_STRAIGHT_AREA[4]   = {2120,   540, 2700, 2630};
int FIRST_GRAY_AREA[4]       = {3920,    0, 4020,  270};
int LUG_AREA[4]              = {4020,    0, 4765,  320}; //lug + back lug area 180624 kota
int BACK_LUG_AREA[4]         = {4390,    0, 4765,  320}; // may not be used
int SECOND_GRAY_AREA[4]      = {4765,    0, 4915,  320};
int SEESAW_AREA[4]           = {   0,    0,    0,    0};
int GARAGE_AREA[4]           = {4915,    0, 5200,  320};

//straigt (x0,y0) - (x1,y1)
//circle (x0,y0,r)
int STRAIGT_01[4] = {   0,  165, 1140,  165};
int CIRCLE_01[3]  = {1140, 605, 440};
int STRAIGT_02[4] = {1580, 605, 1580, 1150};
int CIRCLE_02[3]  = {1105, 1150, 475};
int STRAIGT_03[4] = {1105, 1625, 920, 1625};
int CIRCLE_03[3]  = {920, 1335, 290};

//int STRAIGT_04[4] = {5000, 2790, 3630,  610};
//int CIRCLE_04[3]  = {3870,  450,  290};

int CIRCLE_04[3]  = {500, 890, 305};

//int STRAIGT_05[4] = {3870,  160, 5200,  160};
int CIRCLE_05[3]  = {500, 955, 370};
int CIRCLE_06[3]  = {605, 1150, 475};

int STRAIGT_04[4] = {605, 1625, 1625, 1625};

int CIRCLE_07[3]  = {1625, 1200, 425};
int CIRCLE_08[3]  = {1700, 1090, 350};
int CIRCLE_09[3]  = {2075, 515, 330};
int CIRCLE_10[3]  = {2120, 540, 375};
int STRAIGT_05[4] = {2495, 885,  2495, 2630};


int CIRCLE_01_LENGTH = 1571;

float CIRCLE_01_ANGLE  = 1.570796;

int CIRCLE_02_LENGTH = 707;
float CIRCLE_02_ANGLE  = -1.570796;

int CIRCLE_03_LENGTH = 691;
float CIRCLE_03_ANGLE  = -1.919862;

int CIRCLE_04_LENGTH = 623;
float CIRCLE_04_ANGLE  = 2.146755;

int CIRCLE_05_LENGTH = 0;
float CIRCLE_05_ANGLE  = 0;


int LOG_NAVI = 0;
float FL_LOG = 0.0;
