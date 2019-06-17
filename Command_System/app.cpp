#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "app.hpp"
#include "ev3api.h"

#include "Clock.h"

// デストラクタ問題の回避
// https://github.com/ETrobocon/etroboEV3/wiki/problem_and_coping
void *__dso_handle=0;

// using宣言
//using namespace ev3api;
using ev3api::Clock;

//It will be moved to the class for log 190414 ota
#define LOG_RECORD
#define LOG_SHORT
//#define LOG_LONG
#define LOG_SHORT_SIZE 7500

// Device objects
// オブジェクトを静的に確保する

enum Sys_Mode{
  LINE_TRACE,
  TRACK,
  DEBUG,
};

Sys_Mode SYS_MODE;

Clock* Sys_Clock;

static int32_t   bt_cmd = 0;      /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt     = NULL;   /* Bluetoothファイルハンドル */

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
static int   log_dat_14[LOG_SHORT_SIZE];
static int   log_dat_15[LOG_SHORT_SIZE];
static int   log_dat_16[LOG_SHORT_SIZE];

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

  bool set_mode;

  //**********************************************************************************//
  //Display Course mode on LCD
  //initialize LCD and display Program mode
  //**********************************************************************************//
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("hirojiren_alpha_0525",0, 40);
  //**********************************************************************************//
  //New Object of Sub System(Class)
  //
  //**********************************************************************************//
  // [TODO] タッチセンサの初期化に2msのdelayがあるため、ここで待つ
  tslp_tsk(2);

  
  // オブジェクトの作成
  Sys_Clock       = new Clock();


  //**********************************************************************************//
  //Set Tail Initial position
  //
  //**********************************************************************************//

  //---- 190414 it will be changed later ota
  //  gOperation->tail_reset();
  //  gOperation->tail_stand_up();
  //190414 it will be changed later ota ----

  //**********************************************************************************//
  //Display Robot Status
  //State of Battery, mA,mV, Gyro_offset
  //**********************************************************************************//
  ev3_speaker_set_volume(1);
  ev3_speaker_play_tone(NOTE_C4,200);
  
  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);
  ev3_lcd_draw_string("bluetooh_test",0, 0);
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
  tslp_tsk(1000);

  //**********************************************************************************//
  //Connect Bluetooh
  //
  //**********************************************************************************//
  /* Open Bluetooth file */
  bt = ev3_serial_open_file(EV3_SERIAL_BT);
  assert(bt != NULL);

  /* Bluetooth通信タスクの起動 */
  act_tsk(BT_TASK);

  SYS_MODE = LINE_TRACE;


  ev3_led_set_color(LED_OFF);

  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_speaker_play_tone(NOTE_C4,200);

}


//Systen Destroy
static void sys_destroy(){

}

//It will be move to the class for log ota 20190414 ----
#ifdef LOG_RECORD
static void log_dat( ){
  
  float float_to_int_x1000;

  switch(SYS_MODE){
  case LINE_TRACE:
#ifdef LOG_SHORT
    if (log_cnt < log_size){    
      log_dat_00[log_cnt]  = Sys_Clock->now();
      log_dat_01[log_cnt]  = 0;
      log_dat_02[log_cnt]  = 0;
      log_dat_03[log_cnt]  = 0;
      log_dat_04[log_cnt]  = 0;
      log_dat_05[log_cnt]  = 0;
      log_dat_06[log_cnt]  = 0;
      log_dat_07[log_cnt]  = 0;
      log_dat_08[log_cnt]  = 0;
      log_dat_09[log_cnt]  = 0;
      log_dat_10[log_cnt]  = 0;
      log_dat_11[log_cnt]  = 0;
      log_dat_12[log_cnt]  = 0;
      log_dat_13[log_cnt]  = 0;
      log_dat_14[log_cnt]  = 0;
      log_dat_15[log_cnt]  = 0;
      log_dat_16[log_cnt]  = 0;
    }
      
#endif

#ifdef LOG_LONG
    if (log_cnt < log_size){

    }
#endif
      break;


    case TRACK:
#ifdef LOG_SHORT
      if (log_cnt < log_size){

      }
#endif

#ifdef LOG_LONG
      if (log_cnt < log_size){

      }
#endif

      break;

    case DEBUG:

#ifdef LOG_SHORT

      if (log_cnt < log_size){

      }

#endif

#ifdef LOG_LONG
      if (log_cnt < log_size){

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
  char file_name[MAX_CHAR_NUM] = "log_000_"; //koko
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
      fprintf(fp_wr, "clock, mV, mA, left_motor_pwm, right_motor_pwm, left_motor_enc, right_motor_enc, color_r, color_g, color_b, line_value, x, y, velocity, yaw_rate_x1000, odo, angle_x1000\n");   
      break;

    case TRACK:
      fprintf(fp_wr, "clock, mV, mA, left_motor_pwm, right_motor_pwm, left_motor_enc, right_motor_enc, color_r, color_g, color_b, line_value, x, y, velocity, yaw_rate_x1000, odo, angle_x1000\n");   
      break;

    case DEBUG:
      fprintf(fp_wr, "clock, mV, mA, left_motor_pwm, right_motor_pwm, left_motor_enc, right_motor_enc, color_r, color_g, color_b, line_value, x, y, velocity, yaw_rate_x1000, odo, angle_x1000\n");   
      break;
#endif

#ifdef LOG_LONG
    case LINE_TRACE:
      fprintf(fp_wr, "x,ref_speed,forward,velocity\n");   
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
      fprintf(fp_wr, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",log_dat_00[cnt],log_dat_01[cnt], log_dat_02[cnt],log_dat_03[cnt],log_dat_04[cnt],log_dat_05[cnt],log_dat_06[cnt],log_dat_07[cnt],log_dat_08[cnt],log_dat_09[cnt],log_dat_10[cnt],log_dat_11[cnt],log_dat_12[cnt],log_dat_13[cnt],log_dat_14[cnt],log_dat_15[cnt],log_dat_16[cnt]);
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
    int temp = 0;

    if (ev3_button_is_pressed(ENTER_BUTTON)){
      temp = 9;
      fprintf(bt,"%d",temp);
      fputc(c, bt); /* エコーバック */
    }

    fprintf(bt,"hello");

    switch(c){

    case '0':
      ev3_speaker_play_tone(NOTE_C4,200);
      ev3_led_set_color(LED_GREEN);
      temp = 10;
      fprintf(bt,"%d", temp);
      break;

    case '1':
      ev3_speaker_play_tone(NOTE_D4,200);
      ev3_led_set_color(LED_GREEN);
      bt_cmd = 1;
      temp = 11;
      fprintf(bt,"%d", temp);

      break;

    case '2':
      ev3_speaker_play_tone(NOTE_E4,200);
      ev3_led_set_color(LED_GREEN);
      temp = 12;
      fprintf(bt,"%d", temp);

      break;

    case '3':
      ev3_speaker_play_tone(NOTE_F4,200);
      ev3_led_set_color(LED_GREEN);
      temp = 13;
      fprintf(bt,"%d", temp);

      break;

    case '4':
      ev3_speaker_play_tone(NOTE_G4,200);
      ev3_led_set_color(LED_GREEN);
      temp = 14;
      fprintf(bt,"%d", temp);

      break;


    case '5':
      ev3_speaker_play_tone(NOTE_C4,200);
      ev3_led_set_color(LED_GREEN);
      temp = 15;
      fprintf(bt,"hello");

      break;

    default:
      //      log_dat_00[log_cnt]  = 999;      
      log_cnt++;
      ev3_led_set_color(LED_OFF);
      break;
    }
    fputc(c, bt); /* エコーバック */

  }
}




//Main Task
void main_task(intptr_t unused) {
  //**********************************************************************************//
  //System Intialize
  //**********************************************************************************//
  sys_initialize();


  //---- it will be chaged ota 20190414


  while(1){

    if(ev3_bluetooth_is_connected()){
      ev3_lcd_draw_string("BT connected",0, 60);
    }else{
      ev3_lcd_draw_string("BT unconnected",0, 60);
    }

    if (bt_cmd == 1){

      ev3_led_set_color(LED_GREEN);
    }

    if (ev3_button_is_pressed(BACK_BUTTON)) {
      break;
    }


    tslp_tsk(50); //What dose it mean? kota 170812
    ev3_led_set_color(LED_OFF);
  }

  //  slp_tsk();  // バックボタンが押されるまで待つ
  ev3_lcd_draw_string("Saving Log Data",0, 40);
  export_log_dat( );
  ev3_lcd_draw_string("Saving Log Data is done",0, 60);

  sys_destroy();
  ext_tsk();
}// end::main_task

