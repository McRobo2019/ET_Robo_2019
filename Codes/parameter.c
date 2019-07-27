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
int X_POS_OFFSET = 0;
int Y_POS_OFFSET = 0;


float YAW_LIMIT    = 0.393;   //PI/8 see anago synario
float YAW_STEP     = 0.00786; //YAW_LIMIT/50 see anago synario

//Parameter of Motor CTL 20190719 ota add
float MOTOR_CTL_TS = 0.01;

//float MOTOR_CTL_KI = 0.47782874617737003;
//float MOTOR_CTL_KI = 1.911314984709480; // A
//float MOTOR_CTL_KI = 2.645418663957758; // B
//float MOTOR_CTL_KI = 3.397893306150187; // C
float MOTOR_CTL_KI = 0.012; // test 

//float MOTOR_CTL_KP = 0.0382262996941896;
//float MOTOR_CTL_KP = 0.229357798165138; // A 
//float MOTOR_CTL_KP = 0.296815974096060; // B
//float MOTOR_CTL_KP = 0.356778797145770; // C
float MOTOR_CTL_KP = 0.4013; //test


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

int START_VELOCITY_VAL            = 50; //koko
int FIRST_STRAIGHT_VELOCITY_VAL   = 50;
int ENTER_1ST_CORNER_VELOCITY_VAL = 50;
int FIRST_CORNER_VELOCITY_VAL     = 50;
int SECOND_STRAIGHT_VELOCITY_VAL  = 50;
int ENTER_2ND_CORNER_VELOCITY_VAL = 50;
int SECOND_CORNER_VELOCITY_VAL    = 50;
int THIRD_STRAIGHT_VELOCITY_VAL   = 50;
int THIRD_CORNER_VELOCITY_VAL     = 50;
int S_CORNER_VELOCITY_VAL         = 50;
int FOURTH_STRAIGHT_VELOCITY_VAL  = 50;
int GOAL_VAL                     = 50;

int FOURTH_CORNER_VELOCITY_VAL    = 50;

int ENTER_5TH_CORNER_VELOCITY_VAL = 50;
int FIFTH_CORNER_VELOCITY_VAL     = 50;
int FIRST_GRAY_VELOCITY_VAL       = 50;
int LUG_VELOCITY_VAL              = 50;
int BACK_LUG_VELOCITY_VAL         = 50;
int SECOND_GRAY_VELOCITY_VAL      = 50;
int SEESAW_VELOCITY_VAL           = 50;
int GARAGE_VELOCITY_VAL           = 50;


int START_AREA[4]            = { 300,    0,  500,  320};
int FIRST_STRAIGHT_AREA[4]   = { 500,    0, 2770,  320};
int ENTER_1ST_CORNER_AREA[4] = {2470,    0, 2770,  320};
int FIRST_CORNER_AREA[4]     = {2770,    0, 3780, 1160};
int SECOND_STRAIGHT_AREA[4]  = {3500, 1160, 4000, 2820};
int ENTER_2ND_CORNER_AREA[4] = {3500, 2520, 4000, 2820};
int SECOND_CORNER_AREA[4]    = {3500, 2820, 4220, 4000};
int THIRD_STRAIGHT_AREA[4]   = {4220, 3000, 4660, 4000};
int THIRD_CORNER_AREA[4]     = {4660, 2790, 5500, 4000};
int FOURTH_STRAIGHT_AREA[4]  = {3000,  610, 5500, 2790};
int FOURTH_CORNER_AREA[4]    = {3000,    0, 4000,  610};
int ENTER_5TH_CORNER_AREA[4] = {   0,    0,    0,    0};
int FIFTH_CORNER_AREA[4]     = {   0,    0,    0,    0};
int FIRST_GRAY_AREA[4]       = {3920,    0, 4020,  270};
int LUG_AREA[4]              = {4020,    0, 4765,  320}; //lug + back lug area 180624 kota
int BACK_LUG_AREA[4]         = {4390,    0, 4765,  320}; // may not be used
int SECOND_GRAY_AREA[4]      = {4765,    0, 4915,  320};
int SEESAW_AREA[4]           = {   0,    0,    0,    0};
int GARAGE_AREA[4]           = {4915,    0, 5200,  320};


//straigt (x0,y0) - (x1,y1)
//circle (x0,y0,r)
int STRAIGT_01[4] = {   0,  160, 2770,  160};
int CIRCLE_01[3]  = {2770, 1160, 1000};
int STRAIGT_02[4] = {3770, 1160, 3770, 2820};
int CIRCLE_02[3]  = {4220, 2820,  450};
int STRAIGT_03[4] = {4220, 3270, 4660, 3270};
int CIRCLE_03[3]  = {4660, 2910,  360};
int STRAIGT_04[4] = {5000, 2790, 3630,  610};
//int CIRCLE_04[3]  = {3870,  450,  290};

int CIRCLE_04[3]  = {3970,  450,  300};

int STRAIGT_05[4] = {3870,  160, 5200,  160};
int CIRCLE_05[3]  = {   0,    0,    0};

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


