
/******************************************************************************
 *****************************************************************************/

#ifndef LINE_TRACE_H_
#define LINE_TRACEL_H_

class Line_Trace{

public:
  explicit Line_Trace();
  void init();
  float line_trace_yaw_rate(int line_value, float ref_yaw_rate, float max_yaw_rate, float min_yaw_rate);

private:

  float pos_yaw_step;
  float neg_yaw_step;
  float y_t;
  float target_yaw_rate;
};

#endif  // YAWRATE_CTL_H_
