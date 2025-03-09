/*
 * pid.h
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

 #ifndef PID_H
 #define PID_H
 
 //INTEGRAL TERM IS USE BACKWARD EULER RULE
 typedef struct{
     float kp; //P GAIN FOR PID
     float ki; //I GAIN FOR PID
     float kd; //D GAIN FOR PID
     double y_n; //Initial output
     double y_n_1;
     double e_n; //Initial error
     double e_n_1;
     double e_n_2;
 
 } PID;
 
 void PID_init(PID* pid, float _kp,  float _ki, float _kd);
 double Update_pid(PID *pid, double error, float pid_sat, float plant_sat);
 void Reset_pid(PID* pid);
 #endif /* PID_H */