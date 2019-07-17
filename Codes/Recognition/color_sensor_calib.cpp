#include "color_sensor_calib.hpp"


/**
 * コンストラクタ
 * @param colorSensor カラーセンサ
 */
Color_Sensor_Calib::Color_Sensor_Calib(const ev3api::ColorSensor& colorSensor,
				       ev3api::TouchSensor& touchSensor)
  : 
  mColorSensor(colorSensor),
  mTouch(touchSensor)
{
}

void Color_Sensor_Calib::init(){

  SENSOR_CALIB_MODE = LINE_100;
}

void Color_Sensor_Calib::color_sensor_calib( ) {

  rgb_raw_t rgb_val;
  int       set_value;
  char s[20]={'\0'};

  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
  ev3_lcd_set_font(EV3_FONT_MEDIUM);

  ev3_speaker_set_volume(1);
  ev3_speaker_play_tone(NOTE_C4,200);


  while(1){

    if(SENSOR_CALIB_MODE == CALIB_DONE){
      break;
    }

    switch(SENSOR_CALIB_MODE){
    case LINE_100:

      ev3_lcd_draw_string("Set Line 100%", 0, 20);

      mColorSensor.getRawColor(rgb_val);
      set_value =  rgb_val.b;

      sprintf(s,"BLUE : %d ", rgb_val.b);
      ev3_lcd_draw_string(s, 0, 40);
      ev3_lcd_draw_string("SET    : E",0, 60);
      ev3_lcd_draw_string("CANCEL : TOUCH",0, 80);

      if (mTouch.isPressed()){
	SENSOR_CALIB_MODE = SET_DEFAULT;
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
	ev3_speaker_play_tone(NOTE_E4,400);
      }

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

	if(set_value > CALIB_LINE_100_MAX_THRS ){
	  SENSOR_CALIB_MODE = LINE_100_ERROR;
	  ev3_speaker_play_tone(NOTE_C5,400);
	}else{
	  ev3_speaker_play_tone(NOTE_E4,200);
	  line_0_val=set_value;
	  sprintf(s,"SET_VAL : %d ", line_0_val);
	  ev3_lcd_draw_string(s, 0, 40);
	  //	  SENSOR_CALIB_MODE = LINE_50;
	  SENSOR_CALIB_MODE = LINE_0;
	  tslp_tsk(1000);
	  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
	}
      }

      tslp_tsk(100);
  
      break;

    case LINE_100_ERROR:
      ev3_lcd_draw_string("ERROR Too Big", 0, 20);
      SENSOR_CALIB_MODE = LINE_100;
      tslp_tsk(1000);
      ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
      break;

    case LINE_50:

      ev3_lcd_draw_string("Set Line 50%", 0, 20);
      mColorSensor.getRawColor(rgb_val);
      set_value =  rgb_val.b;

      sprintf(s,"BLUE : %d ", rgb_val.b);
      ev3_lcd_draw_string(s, 0, 40);
      ev3_lcd_draw_string("SET    : E",0, 60);
      ev3_lcd_draw_string("CANCEL : TOUCH",0, 80);

      if (mTouch.isPressed()){
	SENSOR_CALIB_MODE = SET_DEFAULT;
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
      }

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

	if((set_value > CALIB_LINE_50_MAX_THRS)||(set_value < CALIB_LINE_50_MIN_THRS )){
	  SENSOR_CALIB_MODE = LINE_50_ERROR;
	}else{
	  line_50_val = set_value;
	  sprintf(s,"SET_VAL : %d ", line_50_val);
	  ev3_lcd_draw_string(s, 0, 40);
	  SENSOR_CALIB_MODE = LINE_0;
	  tslp_tsk(1000);
	  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
	}
      }

      tslp_tsk(100);

      break;
      
    case LINE_50_ERROR:
      ev3_lcd_draw_string("ERROR Too Big or Small", 0, 20);
      SENSOR_CALIB_MODE = LINE_50;
      tslp_tsk(1000);
      ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
      break;
    
    case LINE_0:
      ev3_lcd_draw_string("Set Line 0%", 0, 20);

      mColorSensor.getRawColor(rgb_val);
      set_value =  rgb_val.b;

      sprintf(s,"BLUE : %d ", rgb_val.b);
      ev3_lcd_draw_string(s, 0, 40);
      ev3_lcd_draw_string("SET    : E",0, 60);
      ev3_lcd_draw_string("CANCEL : TOUCH",0, 80);

      if (mTouch.isPressed()){
	SENSOR_CALIB_MODE = SET_DEFAULT;
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
	ev3_speaker_play_tone(NOTE_E4,400);
      }

      if (ev3_button_is_pressed(ENTER_BUTTON)){
	ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

	if(set_value < CALIB_LINE_0_MIN_THRS ){
	  SENSOR_CALIB_MODE = LINE_0_ERROR;
	  ev3_speaker_play_tone(NOTE_C5,400);
	}else{
	  ev3_speaker_play_tone(NOTE_E4,200);
	  line_100_val = set_value;
	  sprintf(s,"SET_VAL : %d ", line_100_val);
	  ev3_lcd_draw_string(s, 0, 40);
	  SENSOR_CALIB_MODE = SET_GAIN_OFFSET;
	  tslp_tsk(1000);
	  ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
	}
      }

      tslp_tsk(100);
      break;

    case LINE_0_ERROR:
      ev3_lcd_draw_string("ERROR Too Small", 0, 20);
      SENSOR_CALIB_MODE = LINE_0;
      tslp_tsk(1000);
      ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
      break;

    case SET_DEFAULT:
      ev3_lcd_draw_string("Set Defalut", 0, 20);

      line_100_val = 85;
      line_50_val  = 90;
      line_0_val = 15;

      line_val_offset = COLOR_SENSOR_OFFSET;
      line_val_gain   = COLOR_SENSOR_GAIN;

      SENSOR_CALIB_MODE = CALIB_DONE;

      break;

    case SET_GAIN_OFFSET:
      line_val_offset = line_0_val;
      line_val_gain   = line_100_val - line_0_val;
      line_val_gain   = line_val_gain/100.0;
      sprintf(s,"offset: %d ", line_val_offset);
      ev3_lcd_draw_string(s, 0, 40);

      sprintf(s,"gain: %f ", line_val_gain);
      ev3_lcd_draw_string(s, 0, 60);
      SENSOR_CALIB_MODE = CALIB_DONE;
      tslp_tsk(1000);
      break;

    case CALIB_DONE:

      break;

    default:
      break;
    }
  }
}


