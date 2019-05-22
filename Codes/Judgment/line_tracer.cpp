/******************************************************************************
 *  Author: Kaoru Ota
 *****************************************************************************/

#include "line_tracer.hpp"

Line_Trace::Line_Trace(){

}

void Line_Trace::init() {

}

float Line_Trace::line_trace_yaw_rate(int line_value, float ref_yaw_rate, float max_yaw_rate, float min_yaw_rate){

  pos_yaw_step = max_yaw_rate - ref_yaw_rate;
  pos_yaw_step = pos_yaw_step/50.0;

  neg_yaw_step = min_yaw_rate - ref_yaw_rate;
  neg_yaw_step = neg_yaw_step/50.0;

  y_t = (float)line_value-50.0;

  if(y_t >= 0){
    target_yaw_rate = ref_yaw_rate + (y_t * pos_yaw_step);
  }else{
    target_yaw_rate = ref_yaw_rate - (y_t * neg_yaw_step);
  }


  return target_yaw_rate;
}
