#ifndef COLOR_SENSOR_CALIB_H_
#define COLOR_SENSOR_CALIB_H_

#include "ev3api.h"
#include "parameter.h"
#include "ColorSensor.h"
#include "TouchSensor.h"

class Color_Sensor_Calib {
public:
  explicit Color_Sensor_Calib(const ev3api::ColorSensor& colorSensor,
			      ev3api::TouchSensor& touchSensor);
  void  init(); //17.0.28 k-ota add
  void  color_sensor_calib();

  
  int color_r, color_b, color_g;


private:

  static const int8_t INITIAL_WHITE_THRESHOLD;
  static const int8_t INITIAL_BLACK_THRESHOLD;

  const ev3api::ColorSensor& mColorSensor;
  ev3api::TouchSensor& mTouch;

  enum   Sensor_Calib_Mode{
    LINE_100,
    LINE_100_ERROR,
    LINE_50,
    LINE_50_ERROR,
    LINE_0,
    LINE_0_ERROR,
    SET_DEFAULT,
    SET_GAIN_OFFSET,
    CALIB_DONE
  };

  Sensor_Calib_Mode SENSOR_CALIB_MODE;

  int    line_100_val;
  int    line_50_val;
  int    line_0_val;

  int8_t dColor_val[5]; //170814 ota signals for filter of color sensor value.
  
  int   line_val_offset = 0;
  float line_val_gain   = 0.0;



};

#endif  // COLOR_SENSOR_CALIB_H_
