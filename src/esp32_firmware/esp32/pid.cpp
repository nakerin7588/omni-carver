/*
 * pid.c
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */
#include "pid.h"

void PID_init(PID* pid, float _kp,  float _ki, float _kd){
	pid -> kp = _kp;
	pid -> ki = _ki;
	pid -> kd = _kd;
	pid -> y_n = 0.0;
	pid -> y_n_1 = 0.0;
	pid -> e_n = 0.0;
	pid -> e_n_1 = 0.0;
	pid -> e_n_2 = 0.0;
}
double Update_pid(PID *pid, double error, float pid_sat, float plant_sat) {

	float e_n = error; // error[n]

  // if(e_n >= 0.01 && e_n <= 0.01){
  //   pid -> y_n = 0;
  // }

	if(!(((pid -> y_n >= pid_sat) && e_n > 0) || ((pid -> y_n <= -(pid_sat)) && e_n < 0 ))){
		pid -> y_n += ((pid -> kp + pid -> kp + pid -> kp) * e_n)
						- ((pid -> kp + (2 * pid -> kp)) * pid -> e_n_1)
						+ (pid -> kp * pid -> e_n_2);
	}

	if(pid -> y_n >= pid_sat){
		pid -> y_n = pid_sat;

	}else if(pid -> y_n < -pid_sat){
		pid -> y_n = -pid_sat;
	}

	pid -> e_n_2 = pid -> e_n_1;
	pid -> e_n_1 = pid -> e_n;
	pid -> y_n_1 = pid -> y_n;

	return pid -> y_n;
}
void Reset_pid(PID* pid){
	pid -> y_n = 0.0;
	pid -> y_n_1 = 0.0;
	pid -> e_n = 0.0;
	pid -> e_n_1 = 0.0;
	pid -> e_n_2 = 0.0;
}