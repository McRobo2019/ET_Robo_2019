/******************************************************************************
 *****************************************************************************/

#ifndef YAWRATE_CTL_H_
#define YAWRATE_CTL_H_

class Yawrate_Ctl{

public:
  explicit Yawrate_Ctl();
  void init();
  float YawrateController(float yawrate, float yawrate_cmd);

private:

  float yaw_ctl_dt = 0.004;

  float turn_val   = 0.0;
  float r_yaw_rate = 0.0;
  float r_yaw_rate_ud = 0.0;
  float I_gain1 = 1.0+1.0/0.8;
  float I_gain2 = yaw_ctl_dt/0.06/0.8;
  float I_gain3 = 1.0-yaw_ctl_dt/0.06;

  float F_controller(float r_yaw_rate);
  float F_in = 0.0;
  float F_out = 0.0;
  float F_gain = 1/0.062;
    
  float E_controller(float r_yaw_rate);
  float E_in;
  float E_in_d;
  float E_in_dd;
  float E_in_ddd;
  float E_in_dddd;
  float E_in_ddddd = 0.0;
  float E_in_dddddd = 0.0;

  float E_out = 0.0;
  float E_gain1 = yaw_ctl_dt/0.1;
  float E_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
  float E_ud1 = 0.0;

  float C_controller(float E_out, float yawrate, float S_out);
  float C_in = 0.0;
  float C_out = 0.0;
  float C_gain = yaw_ctl_dt*50.0;
  float C_ud1 = 0.0;

  float S_controller(float C_out);
  float S_gain1 = 1.0;
  float S_in;
  float S_in_d;
  float S_in_dd;
  float S_in_ddd;
  float S_in_dddd;
  float S_in_ddddd = 0.0;
  float S_in_dddddd = 0.0;
  float S_out = 0.0;
  float Pn_gain1 = yaw_ctl_dt/0.1;
  float Pn_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
  float Pn_ud1 = 0.0;
  float Pd_gain1 = yaw_ctl_dt/0.1;
  float Pd_gain2 = 1.0-yaw_ctl_dt/0.1*1.0;
  float Pd_ud1 = 0.0;
};

#endif  // YAWRATE_CTL_H_
