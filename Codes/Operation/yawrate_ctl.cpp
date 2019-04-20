/******************************************************************************
 *  Author: Kaoru Ota
 *****************************************************************************/

#include "yawrate_ctl.hpp"

//using ev3api::Clock;

//Clock*       robo_Clock;

Yawrate_Ctl::Yawrate_Ctl(){

}

void Yawrate_Ctl::init() {

  //  robo_Clock       = new Clock();        


}

float Yawrate_Ctl::YawrateController(float yawrate, float yawrate_cmd)
{
  //r_yaw_rate = yawrate_cmd*I_gain1 - r_yaw_rate_ud; //20180530 kota
  r_yaw_rate = (-1.0*yawrate_cmd*I_gain1) - r_yaw_rate_ud; //20180530 kota
  
  r_yaw_rate_ud =(r_yaw_rate*I_gain2 + r_yaw_rate_ud*I_gain3);
  F_out = F_controller((float)r_yaw_rate);
  E_out = E_controller((float)r_yaw_rate);
  C_out = C_controller(E_out, (float)yawrate, S_out);
  S_out = S_controller(C_out);
  turn_val = F_out + C_out;
  if(turn_val > 100) turn_val = 100;
  if(turn_val < -100) turn_val = -100;
  
  return turn_val;//制御出力

}

float Yawrate_Ctl::F_controller(float r_yaw_rate)
{
	F_in = r_yaw_rate;

	F_out = F_in * F_gain;

	return F_out;
}

float Yawrate_Ctl::E_controller(float r_yaw_rate)
{
	E_in_dddddd = E_in_ddddd;
	E_in_ddddd = E_in_dddd;
	E_in_dddd = E_in_ddd;
	E_in_ddd = E_in_dd;
	E_in_dd = E_in_d;
	E_in_d = E_in;
	E_in = (r_yaw_rate);
	E_out = E_ud1;
	E_ud1 = (E_in_dddddd * E_gain1) + E_ud1 * E_gain2;

	return E_out;
}

float Yawrate_Ctl::C_controller(float E_out, float yawrate, float S_out)
{
	C_in = (E_out + yawrate - S_out);
	C_out = C_ud1;
	C_ud1 = C_in * C_gain + (C_out * 1.0);

	if(C_ud1 > 20){
	  C_ud1 = 20;
	}else if(C_ud1 < -20){
	  C_ud1 = -20;
	}

	return C_out;
}

float Yawrate_Ctl::S_controller(float C_out)
{
	S_in_dddddd = S_in_ddddd;
	S_in_ddddd = S_in_dddd;
	S_in_dddd = S_in_ddd;
	S_in_ddd = S_in_dd;
	S_in_dd = S_in_d;
	S_in_d = S_in;
	S_in = C_out*S_gain1;
	S_out = Pn_ud1 - Pd_ud1;
	Pn_ud1 = S_in*Pn_gain1 + Pn_ud1*Pn_gain2;
	Pd_ud1 = S_in_dddddd*Pd_gain1 + Pd_ud1*Pd_gain2;

	return S_out;
}


