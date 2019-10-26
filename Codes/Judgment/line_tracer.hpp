
/******************************************************************************
 *****************************************************************************/

#ifndef LINE_TRACE_H_
#define LINE_TRACEL_H_

#include "util.hpp"
#include "parameter.h"


class Line_Trace{

public:
  explicit Line_Trace();
  void init();
  float line_trace_omega(int line_value, float ref_omega, float max_omega, float min_omega);
  
  PID *gLine_trace_PID = new PID();
  float kp;
  float ki;
  float kd;


private:

  float pos_omega_step;
  float neg_omega_step;
  float y_t;
  float target_omega;
};

#endif
