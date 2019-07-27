/******************************************************************************
 *  Author: Kaoru Ota
 *****************************************************************************/

#include "line_tracer.hpp"

Line_Trace::Line_Trace(){

}

void Line_Trace::init() {
  kp = 5.0;
  ki = 0;
  kd = 1.0;
  gLine_trace_PID->init_pid(kp, ki, kd, dT_10ms);
}

float Line_Trace::line_trace_omega(int line_value, float ref_omega, float max_omega, float min_omega){

  pos_omega_step = max_omega - ref_omega;
  pos_omega_step = pos_omega_step/50.0;
  //pos_omega_step = pos_omega_step/100.0;
    
  neg_omega_step = min_omega - ref_omega;
  neg_omega_step = neg_omega_step/50.0;  
  //neg_omega_step = neg_omega_step/100.0;

  y_t = (float)line_value-50.0;
  //  y_t = gLine_trace_PID->calc_pid(50.0, line_value);

  if(y_t >= 0){
    target_omega = ref_omega + (y_t * pos_omega_step);
  }else{
    target_omega = ref_omega - (y_t * neg_omega_step);
  }


  return target_omega;
}
