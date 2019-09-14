//Hirojiren Proto System
//Date:2019.4.13
//Author:Kaoru Ota
//git@github.com:McRobo2019/hiro_jiren.git

#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "app.hpp"
#include "util.hpp"
#include "ev3api.h"

#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"

//sub systemp
#include "color_sensor_calib.hpp"
#include "recognition.hpp"
#include "judgment.hpp"
#include "operation.hpp"

// デストラクタ問題の回避
// https://github.com/ETrobocon/etroboEV3/wiki/problem_and_coping
void *__dso_handle=0;

// using宣言
//using namespace ev3api;
using ev3api::ColorSensor;
using ev3api::GyroSensor;
using ev3api::TouchSensor;
using ev3api::SonarSensor;
using ev3api::Motor;

//It will be moved to the class for log 190414 ota
#define LOG_RECORD
#define LOG_SHORT
//#define LOG_LONG
#define LOG_SHORT_SIZE 10000

// Device objects
// オブジェクトを静的に確保する
TouchSensor gTouchSensor (PORT_1);
ColorSensor gColorSensor (PORT_2);
GyroSensor  gGyroSensor  (PORT_3);
SonarSensor gSonarSensor (PORT_4);

Motor       gRightMotor  (PORT_A);
Motor       gTailMotor   (PORT_B);
Motor       gArmMotor    (PORT_C);
Motor       gLeftMotor   (PORT_D);


enum Sys_Mode{
  LINE_TRACE,
  TRACK,
  DEBUG,
};
Sys_Mode SYS_MODE;

/* コマンドコードの定義 */
enum ECommandCode {
  eCap = 0x10,
};

/* レスポンスコードの定義 */
enum EResCode {
  eCap_img = 0x1f,
  eCommand = 0x2f
};




static int32_t   bt_cmd = 0;      /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt     = NULL;   /* Bluetoothファイルハンドル */

static Color_Sensor_Calib   *gColor_Sensor_Calib;
static Recognition          *gRecognition;
static Judgment             *gJudgment;
static Operation            *gOperation;

static  uint8_t rescode;
static  uint8_t *response   = NULL;
static  size_t  response_len = 0;



//It will be moved to log class 190414 ota ----
#ifdef LOG_RECORD

#ifdef LOG_SHORT
static int   log_size = LOG_SHORT_SIZE;
static int   log_cnt  = 0;
static int   log_dat_00[LOG_SHORT_SIZE];
static int   log_dat_01[LOG_SHORT_SIZE];
static int   log_dat_02[LOG_SHORT_SIZE];
static int   log_dat_03[LOG_SHORT_SIZE];
static int   log_dat_04[LOG_SHORT_SIZE];
static int   log_dat_05[LOG_SHORT_SIZE];
static int   log_dat_06[LOG_SHORT_SIZE];
static int   log_dat_07[LOG_SHORT_SIZE];
static int   log_dat_08[LOG_SHORT_SIZE];
static int   log_dat_09[LOG_SHORT_SIZE];
static int   log_dat_10[LOG_SHORT_SIZE];
static int   log_dat_11[LOG_SHORT_SIZE];
static int   log_dat_12[LOG_SHORT_SIZE];
static int   log_dat_13[LOG_SHORT_SIZE];
/*
static int   log_dat_14[LOG_SHORT_SIZE];
static int   log_dat_15[LOG_SHORT_SIZE];
static int   log_dat_16[LOG_SHORT_SIZE];
*/

static float   float_log_00[LOG_SHORT_SIZE];
static float   float_log_01[LOG_SHORT_SIZE];
static float   float_log_02[LOG_SHORT_SIZE];
static float   float_log_03[LOG_SHORT_SIZE];


#endif

#ifdef LOG_LONG
static int   log_size = 20000;
static int   log_cnt  = 0;
static int   log_dat_00[20000];
static int   log_dat_01[20000];
static int   log_dat_02[20000];
static int   log_dat_03[20000];
#endif

#endif

//----It will be moved to log class 190414 ota


/****************************************************************************************/
//System Initialize
//
/****************************************************************************************/

static void sys_initialize() {
  
  int  battery;
  char battery_str[32];
  int  gyro;
  char gyro_str[32];
  bool set_mode;

  //**********************************************************************************//
  //Display Course mode on LCD
  //initialize LCD and display Program mode
  //**********************************************************************************//
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("Line_Trace",0, 40);
  //**********************************************************************************//
  //New Object of Sub System(Class)
  //
  //**********************************************************************************//
  // [TODO] タッチセンサの初期化に2msのdelayがあるため、ここで待つ
  tslp_tsk(2);

  
  // オブジェクトの作成
  gColor_Sensor_Calib = new Color_Sensor_Calib(gColorSensor,
				 gTouchSensor);

  gRecognition = new Recognition(gColorSensor,
				 gLeftMotor,
				 gRightMotor,
				 gGyroSensor,
				 gSonarSensor);

  gJudgment    = new Judgment();
  gOperation   = new Operation(gGyroSensor,
			       gLeftMotor,
			       gRightMotor,
			       gArmMotor,
			       gTailMotor);

  //**********************************************************************************//
  //Display Robot Status
  //State of Battery, mA,mV, Gyro_offset
  //**********************************************************************************//
  ev3_speaker_set_volume(1);
  ev3_speaker_play_tone(NOTE_C4,200);
  
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("LEFT-0915_0",0, 0);
  battery = ev3_battery_voltage_mV();
  sprintf(battery_str, "V:%d", battery);
  ev3_lcd_draw_string(battery_str,0, 20);

  battery = ev3_battery_current_mA();
  sprintf(battery_str, "A:%d", battery);
  ev3_lcd_draw_string(battery_str,0, 40);

  //**********************************************************************************//
  //Set Gyro offset
  //
  //**********************************************************************************//
  ev3_lcd_draw_string("Put on GND",0, 60);
  ev3_lcd_draw_string("PUSH TS 4 RESET",0, 80);

  while(1){
    if (gTouchSensor.isPressed()){
      gColor_Sensor_Calib->init();
      gRecognition->init();   //reset gyro
      gOperation->init();  //
      gJudgment->init(); //initialize mode
      break; /* タッチセンサが押された */
    }
    gyro = gGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    sprintf(gyro_str, "Gyro:%d", gyro);
    ev3_lcd_draw_string(gyro_str,0, 100);
    tslp_tsk(10);
  }
  ev3_speaker_play_tone(NOTE_E4,200);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

  tslp_tsk(500);

  while(1){
    if (gTouchSensor.isPressed()){
      break; /* タッチセンサが押された */
    }
    gyro = gGyroSensor.getAnglerVelocity();  // ジャイロセンサ値
    sprintf(gyro_str, "Gyro:%d", gyro);
    ev3_lcd_draw_string(gyro_str,0, 100);
    tslp_tsk(20);
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  }
  ev3_speaker_play_tone(NOTE_E4,200);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

  //**********************************************************************************//
  //Connect Bluetooh
  //
  //**********************************************************************************//
  /* Open Bluetooth file */
  bt = ev3_serial_open_file(EV3_SERIAL_BT);
  assert(bt != NULL);

  /* Bluetooth通信タスクの起動 */
  act_tsk(BT_TASK);

  //**********************************************************************************//
  //Select Mode
  //
  //**********************************************************************************//
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("Select Mode",0, 0);

  SYS_MODE = LINE_TRACE;

  while(1){
    set_mode = false;

    switch(SYS_MODE){

    case LINE_TRACE:
      ev3_lcd_draw_string("->LINE_TRACE",0, 20);
      ev3_lcd_draw_string("TRACK_MODE  ",0, 40);
      ev3_lcd_draw_string("DEBUG_MODE  ",0, 80);

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_speaker_play_tone(NOTE_E4,200);
	SYS_MODE = LINE_TRACE;
	set_mode = true;
      }else if (ev3_button_is_pressed(DOWN_BUTTON)){
	SYS_MODE = TRACK;
	set_mode = false;
      }else{
	SYS_MODE = LINE_TRACE;
	set_mode = false;
      }
      break;


    case TRACK:
      ev3_lcd_draw_string("LINE_TRACE  ",0, 20);
      ev3_lcd_draw_string("->TRACK_MODE",0, 40);
      ev3_lcd_draw_string("DEBUG_MODE  ",0, 80);

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_speaker_play_tone(NOTE_E4,200);
	SYS_MODE = TRACK;
	set_mode = true;
      }else if (ev3_button_is_pressed(DOWN_BUTTON)){
	SYS_MODE = DEBUG;
	set_mode = false;
      }else{
	SYS_MODE = TRACK;
	set_mode = false;
      }
      break;

    case DEBUG:
      ev3_lcd_draw_string("LINE_TRACE  ",0, 20);
      ev3_lcd_draw_string("TRACK_MODE  ",0, 40);
      ev3_lcd_draw_string("->DEBUG_MODE",0, 80);

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_speaker_play_tone(NOTE_E4,200);
	SYS_MODE = DEBUG;
	set_mode = true;
      }else if (ev3_button_is_pressed(DOWN_BUTTON)){
	SYS_MODE = LINE_TRACE;
	set_mode = false;
      }else{
	SYS_MODE = DEBUG;
	set_mode = false;
      }
      break;

    default:
      SYS_MODE = LINE_TRACE;
      set_mode = false;
      break;
    }

    tslp_tsk(100);

    
    if (set_mode == true){

      if(SYS_MODE == LINE_TRACE){
	ev3_lcd_draw_string("SET_LINE_TRACE            ",0, 100);
	gJudgment->set_drive_mode_LT();
      }else if(SYS_MODE == TRACK){
	ev3_lcd_draw_string("SET_TRACK_MODE            ",0, 100);
	gJudgment->set_drive_mode_TK();
      }else{
	ev3_lcd_draw_string("SET_DEBUG_MODE            ",0, 100);
	gJudgment->set_drive_mode_DB();
      }
      tslp_tsk(1000);
      break;
    }else{
      ev3_lcd_draw_string("select mode down and enter",0, 100);
    }
    
  }

  //**********************************************************************************//
  //Completed Intitialize
  //**********************************************************************************//
  // 初期化完了通知
  ev3_led_set_color(LED_OFF);

  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_speaker_play_tone(NOTE_C4,200);

}


//Systen Destroy
static void sys_destroy(){
  delete gRecognition;
  delete gJudgment;
  delete gOperation;
}


/* コマンド(要求)パケット生成関数 */
static size_t encode_packet(uint8_t reqcode, uint16_t points[], size_t num_of_points, uint8_t **packet)
{
	uint8_t *data = NULL;
	size_t l_parameter_length = 0;
	size_t packet_size = 0;
	size_t packet_index = 0;

	if ( (packet == NULL) || (reqcode != eCap) || (num_of_points > 84) ) {
	  return 0;
	}

	packet_size = 1/* Command Code */ + 1/* Parameter Length */ + l_parameter_length;

	data = (uint8_t *)malloc( packet_size );
	if ( data == NULL ) {
	  return 0;
	}
	
	/* command code */
	data[packet_index++] = reqcode;
	
	/* parameter length */
	data[packet_index++] = l_parameter_length;
	
	/* parameter */
	*packet = data;

	return packet_size;
}

static int serial_write(FILE *bt, uint8_t *data, size_t data_len)
{
	if ( (bt == NULL) || (data == NULL) ) {
		return -1;
	}

	if ( fwrite(data, 1, data_len, bt) < data_len ) {
		return -1;
	}

	return 0;
}

static int serial_read(FILE *bt, uint8_t *code, uint8_t **data, size_t *data_len)
{
	uint8_t l_code;
	uint8_t l_data_len;
	uint8_t *l_data = NULL;

	if ( (bt == NULL) || (code == NULL) || (data == NULL) || (data_len == NULL) ) {
		return -1;
	}

	if ( fread(&l_code, sizeof(uint8_t), 1, bt) < 1 ) {
		return -1;
	}
	
	if ( fread(&l_data_len, sizeof(uint8_t), 1, bt) < 1 ) {
		return -1;
	}

	l_data = (uint8_t *)malloc(l_data_len);
	if ( l_data == NULL ) {
		return -1;
	}
	if ( fread(l_data, sizeof(uint8_t), l_data_len, bt) < l_data_len ) {
		free(l_data);
		return -1;
	}

	*code = l_code;
	*data_len = l_data_len;
	*data = l_data;

	return 0;
}





//It will be move to the class for log ota 20190414 ----
#ifdef LOG_RECORD
static void log_dat( ){
  
  //  float float_to_int_x1000;

  switch(SYS_MODE){
  case LINE_TRACE:
#ifdef LOG_SHORT
    if (log_cnt < log_size){    
	log_dat_00[log_cnt]  = SYS_CLK;

	log_dat_01[log_cnt]  = LOG_NAVI;
	log_dat_02[log_cnt]  = LINE_VAL;	    

	//	log_dat_03[log_cnt]  = gOperation->left_motor_pwm;
	//	log_dat_04[log_cnt]  = gOperation->right_motor_pwm;

	log_dat_03[log_cnt]  = (int)X_POS;
	log_dat_04[log_cnt]  = (int)Y_POS;	    

	log_dat_05[log_cnt]  = gRecognition->xvalue;
	log_dat_06[log_cnt]  = gRecognition->yvalue;

	log_dat_07[log_cnt]  = gJudgment->target_velocity;  //20190620 ota
	log_dat_08[log_cnt]  = (int)ODO;
	log_dat_09[log_cnt]  = gOperation->target_left_velocity;
	log_dat_10[log_cnt]  = gOperation->target_right_velocity;

	log_dat_11[log_cnt]  = (int)gRecognition->velocity;
	log_dat_12[log_cnt]  = (int)gRecognition->left_wheel_velocity;
	log_dat_13[log_cnt]  = (int)gRecognition->right_wheel_velocity;
	
	float_log_00[log_cnt] = FL_LOG;
	float_log_01[log_cnt] = YAW_ANGLE;
	float_log_02[log_cnt] = gJudgment->target_omega;
	float_log_03[log_cnt] = gRecognition->omega;

	//      log_dat_01[log_cnt]  = ev3_battery_voltage_mV();
	//	log_dat_07[log_cnt]  = gRecognition->color_r;
	//      log_dat_08[log_cnt]  = gRecognition->color_g;
	//      log_dat_09[log_cnt]  = gRecognition->color_b;
    }
      
#endif

#ifdef LOG_LONG
    if (log_cnt < log_size){
      log_dat_00[log_cnt]  = (int)X_POS;
      log_dat_01[log_cnt]  = (int)Y_POS;
      log_dat_02[log_cnt]  = gRecognition->velocity;
      log_dat_03[log_cnt] =  (int)(YAW_ANGLE*1000.0);
    }
#endif
      break;


    case TRACK:
#ifdef LOG_SHORT
      if (log_cnt < log_size){
	log_dat_00[log_cnt]  = SYS_CLK;
	log_dat_01[log_cnt]  = LOG_NAVI;
	log_dat_02[log_cnt]  = LINE_VAL;	    
	log_dat_03[log_cnt]  = (int)X_POS;
	log_dat_04[log_cnt]  = (int)Y_POS;	    
	log_dat_05[log_cnt]  = gRecognition->xvalue;
	log_dat_06[log_cnt]  = gRecognition->yvalue;
	log_dat_07[log_cnt]  = gJudgment->target_velocity;  //20190620 ota
	log_dat_08[log_cnt]  = (int)ODO;
	log_dat_09[log_cnt]  = gOperation->target_left_velocity;
	log_dat_10[log_cnt]  = gOperation->target_right_velocity;
	log_dat_11[log_cnt]  = (int)gRecognition->velocity;
	log_dat_12[log_cnt]  = (int)gRecognition->left_wheel_velocity;
	log_dat_13[log_cnt]  = (int)gRecognition->right_wheel_velocity;
	float_log_00[log_cnt] = FL_LOG;
	float_log_01[log_cnt] = YAW_ANGLE;
	float_log_02[log_cnt] = gJudgment->target_omega;
	float_log_03[log_cnt] = gRecognition->omega;

	//      log_dat_01[log_cnt]  = ev3_battery_voltage_mV();
	//	log_dat_07[log_cnt]  = gRecognition->color_r;
	//      log_dat_08[log_cnt]  = gRecognition->color_g;
	//	log_dat_02[log_cnt]  = ev3_battery_current_mA();
	//	log_dat_03[log_cnt]  = gOperation->left_motor_pwm;
	//	log_dat_04[log_cnt]  = gOperation->right_motor_pwm;
	//	log_dat_05[log_cnt]  = gRecognition->encL;
	//	log_dat_06[log_cnt]  = gRecognition->encR;
	//	log_dat_07[log_cnt]  = gJudgment->target_velocity;  //20190620 ota
	//	float_log_00[log_cnt] = gRecognition->yawrate;
      }
#endif

#ifdef LOG_LONG
      if (log_cnt < log_size){
	log_dat_00[log_cnt]  = (int)X_POS;
	log_dat_01[log_cnt]  = gJudgment->forward;
	log_dat_03[log_cnt]  = gRecognition->velocity;
      }
#endif

      break;

    case DEBUG:

#ifdef LOG_SHORT

      if (log_cnt < log_size){
	log_dat_00[log_cnt]  = SYS_CLK;
	log_dat_01[log_cnt]  = LOG_NAVI;
	log_dat_02[log_cnt]  = ev3_battery_current_mA();

	log_dat_03[log_cnt]  = gOperation->left_motor_pwm;
	log_dat_04[log_cnt]  = gOperation->right_motor_pwm;

	log_dat_05[log_cnt]  = gRecognition->encL;
	log_dat_06[log_cnt]  = gRecognition->encR;

	log_dat_07[log_cnt]  = gJudgment->target_velocity;  //20190620 ota
	log_dat_08[log_cnt]  = gJudgment->target_omega;
	log_dat_09[log_cnt]  = gOperation->target_left_velocity;
	log_dat_10[log_cnt]  = gOperation->target_right_velocity;

	log_dat_11[log_cnt]  = (int)gRecognition->velocity;
	log_dat_12[log_cnt]  = (int)gRecognition->left_wheel_velocity;
	log_dat_13[log_cnt]  = (int)gRecognition->right_wheel_velocity;

	/*
	float_to_int_x1000   = gRecognition->yawrate * 1000.0;
	log_dat_14[log_cnt]  =  (int)float_to_int_x1000;

	log_dat_15[log_cnt]  = gRecognition->odo;
      
	float_to_int_x1000   =  YAW_ANGLE*1000.0;
	log_dat_16[log_cnt]  =  (int)float_to_int_x1000;
	*/
	
	float_log_00[log_cnt] = gRecognition->yawrate;
	float_log_01[log_cnt] = YAW_ANGLE;
	float_log_02[log_cnt] = gJudgment->target_omega;
	float_log_03[log_cnt] = gRecognition->omega;
      }

#endif

#ifdef LOG_LONG
      if (log_cnt < log_size){
	log_dat_00[log_cnt]  = (int)X_POS;
	log_dat_01[log_cnt]  = gJudgment->forward;
	log_dat_03[log_cnt]  = gRecognition->velocity;
      }
#endif

      break;

  default:

      break;
  }

  if (log_cnt < log_size){
    log_cnt++;
  }
}

//It will be move to the class for log ota 20190414
static void export_log_dat( ){

#define MAX_CHAR_NUM 100
#define CHAR_ARRAY_NUM 2

  //  FILE* file_id;
  //    int battery = ev3_battery_voltage_mV();

  FILE  *fp_rd;
  FILE  *fp_wr;

  char word_array[CHAR_ARRAY_NUM][MAX_CHAR_NUM];
	
  int i;
		
  //file name gen ----
  char file_name[MAX_CHAR_NUM] = "left_"; //koko
  char file_cnt[MAX_CHAR_NUM] = "000";
  char file_format[MAX_CHAR_NUM] = ".csv";
  int  str_to_num;
  //---- file name gen


	//READ FILE NUMBER
  fp_rd = fopen("log_num.csv", "r");


  i = 0;
  while (fgets(&word_array[i][0], MAX_CHAR_NUM, fp_rd) != NULL){
    i++;
  }
  fclose(fp_rd);
  
  //incremant number of dat file ----
  str_to_num = atoi(&word_array[0][0]);
  str_to_num = str_to_num + 10;
  sprintf(file_cnt, "%d", str_to_num);
  strcat(file_name, file_cnt);
  strcat(file_name, file_format);
	
  //UPDATA NUM in sys_dat
  fp_wr = fopen("log_num.csv", "w");
  fprintf(fp_wr, "%d\n", str_to_num);
  fclose(fp_wr);

  //LOG DATA WRITE
  fp_wr = fopen(file_name, "w");
  



  switch(SYS_MODE){
#ifdef LOG_SHORT
    case LINE_TRACE:
      fprintf(fp_wr, "clock, log_navi, line, x, y, left_motor_enc, right_motor_enc, target_velocity, odo, target_vl, target_vr, velocity, left_wheel_velo, right_wheel_velo, fl_log, abs_angle, target_omega, omega\n");   
      break;

    case TRACK:
      fprintf(fp_wr, "clock, log_navi, line, x, y, left_motor_enc, right_motor_enc, target_velocity, odo, target_vl, target_vr, velocity, left_wheel_velo, right_wheel_velo, fl_log, abs_angle, target_omega, omega\n");   
      break;

    case DEBUG:
      fprintf(fp_wr, "clock, navi_log, mA, left_motor_pwm, right_motor_pwm, left_motor_enc, right_motor_enc, target_velocity, target_omega, target_vl, target_vr, velocity, left_wheel_velo, right_wheel_velo, yaw_rate, abs_angle, target_omega, omega\n");   
      break;
#endif

#ifdef LOG_LONG
    case LINE_TRACE:
      fprintf(fp_wr, "x,y,velocity,anglex1000\n");   
      break;

    case TRACK:
      fprintf(fp_wr, "x,ref_speed,forward,velocity\n");   
      break;

    case DEBUG:
      fprintf(fp_wr, "x,ref_speed,forward,velocity\n");   
#endif

  default:
      break;
  }

    int cnt;

    for(cnt = 0; cnt < log_size ; cnt++){
#ifdef LOG_SHORT
      fprintf(fp_wr, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f, %f\n",log_dat_00[cnt],log_dat_01[cnt], log_dat_02[cnt],log_dat_03[cnt],log_dat_04[cnt],log_dat_05[cnt],log_dat_06[cnt],log_dat_07[cnt],log_dat_08[cnt],log_dat_09[cnt],log_dat_10[cnt],log_dat_11[cnt],log_dat_12[cnt],log_dat_13[cnt],float_log_00[cnt], float_log_01[cnt], float_log_02[cnt], float_log_03[cnt]);
#endif

#ifdef LOG_LONG
      fprintf(fp_wr, "%d,%d,%d,%d\n",log_dat_00[cnt],log_dat_01[cnt], log_dat_02[cnt],log_dat_03[cnt]);
#endif

    }
    fclose(fp_wr);
}
#endif



//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
  while(1){
    uint8_t c = fgetc(bt); /* 受信 */


    switch(c){
    case '1':
      bt_cmd = 1;
      break;

    case '0':
      ev3_speaker_play_tone(NOTE_C4,200);
      ev3_led_set_color(LED_GREEN);
      break;

    default:
      ev3_led_set_color(LED_OFF);
      break;
    }
    fputc(c, bt); /* エコーバック */

  }
}

void rec_cyc(intptr_t exinf) {
    act_tsk(REC_TASK);
}

void rec_task(intptr_t exinf) {

    gRecognition->run();

    if(gJudgment->line_trace_mode){
      //      gRecognition->correct_odometry();
    }

    gRecognition->setSonarDistance();

    gJudgment->set_in_data(gRecognition->green_flag,
			   gRecognition->velocity,
			   gRecognition->pre_velo_0p5sec, 
			   gRecognition->yawrate,
			   gRecognition->ave_angle,
			   gTailMotor.getCount(),             //it will be modified later 20190420 ota
			   gRecognition->robo_stop,
			   gRecognition->robo_forward,
			   gRecognition->robo_back,
			   gRecognition->robo_turn_left,
			   gRecognition->robo_turn_right,
			   gRecognition->sonarDistance);
    
  ext_tsk();
}


void jud_cyc(intptr_t exinf) {
    act_tsk(JUD_TASK); //0817 tada
}

void jud_task(intptr_t exinf) {
  int ret = -1;
  uint8_t *request = NULL;
  size_t request_len = 0;

  gJudgment->run(response, response_len);
  gOperation->setCommand(gRecognition->ave_velo,//gRecognition->velocity,
			 gRecognition->left_wheel_velocity,
			 gRecognition->right_wheel_velocity,
			 gJudgment->forward,
			 gJudgment->target_yaw_rate,
			 gRecognition->yawrate,
			 gJudgment->target_velocity,
			 gJudgment->target_omega
			 );


  if(BLOCK_MODE){
    if(RECEIVED_CMD == false){
      request_len = encode_packet(eCap, NULL, 0, &request);
      serial_write(bt, request, request_len);
      ret = serial_read(bt, &rescode, &response, &response_len);
      if(ret == 0){
	  RECEIVED_CMD = true;
	}
    }
  }

    ext_tsk();
}

void ope_cyc(intptr_t exinf) {
    act_tsk(OPE_TASK);
}

void ope_task(intptr_t exinf) {

#ifdef LOG_RECORD
  log_dat();
#endif

#ifdef LOG_RECORD  
  if (ev3_button_is_pressed(DOWN_BUTTON)){
    wup_tsk(MAIN_TASK);
  }
#endif

  if (ev3_button_is_pressed(BACK_BUTTON)) {
    wup_tsk(MAIN_TASK);  // バックボタン押下
  } else {
    gOperation->run();
  }
  ext_tsk();
}


//Main Task
void main_task(intptr_t unused) {
  //  int ret = -1;
  uint8_t *request = NULL;
  size_t request_len = 0;
  //**********************************************************************************//
  //System Intialize
  //**********************************************************************************//
  sys_initialize();
  //**********************************************************************************//
  //Reset angle of arm
  //**********************************************************************************//
  gOperation->arm_reset();
  gOperation->arm_line_trace();
  //**********************************************************************************//
  //Color Sensor calibration 
  //20190721 move sensoor_calib functin from rec class to color_sensor_calib class
  //**********************************************************************************//
  gColor_Sensor_Calib->color_sensor_calib();
  delete gColor_Sensor_Calib; //delete the class after calib

  //REDAY for START
  ev3_sta_cyc(REC_CYC);
  
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("Set ANG on Start Line",0, 40);
  ev3_lcd_draw_string("PRESS TS or 1",0, 80);

  /* DEBUG SERIAL ************/
  uint8_t   *data;
  const int SIZE = 16;
  uint8_t uint8_dat;
  int i;

  uint8_dat = 0x00;
  data = new uint8_t[SIZE];

  for (i=0; i < SIZE; i++){
    data[i] = uint8_dat;
    uint8_dat = uint8_dat + 0x01;
    
  }
  response     = data;
  response_len = SIZE;
  /************ DEBUG SERIAL */

  while(1){
    if(ev3_bluetooth_is_connected()){
      ev3_lcd_draw_string("BT connected",0, 60);
    }else{
      ev3_lcd_draw_string("BT unconnected",0, 60);
    }

    if (gTouchSensor.isPressed()){
      tslp_tsk(100);
      break; /* タッチセンサが押された */
    }
    tslp_tsk(10); //What dose it mean? kota 170812
  }

  // メッセージ Command Systemを送信
  request_len = encode_packet(eCap, NULL, 0, &request);
  //  ret = serial_write(bt, request, request_len);
  serial_write(bt, request, request_len);

  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_led_set_color(LED_OFF);

  ev3_sta_cyc(JUD_CYC);
  gOperation->set_robo_mode_launch();
  ev3_sta_cyc(OPE_CYC);
  //  ter_tsk(BT_TASK);

  slp_tsk();  // バックボタンが押されるまで待つ

  ev3_stp_cyc(OPE_CYC);

  gLeftMotor.~Motor();
  gRightMotor.~Motor();
  gArmMotor.~Motor();
  gTailMotor.~Motor();

  ev3_led_set_color(LED_ORANGE);
  ev3_lcd_set_font(EV3_FONT_SMALL);
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_draw_string("Stop",0, 20);

#ifdef LOG_RECORD
  ev3_lcd_draw_string("Saving Log Data",0, 40);
  export_log_dat( );
  ev3_lcd_draw_string("Saving Log Data is done",0, 60);
#endif

  ev3_led_set_color(LED_OFF);



  sys_destroy();
  ext_tsk();
}// end::main_task

