/*
 * pid.h
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

 #ifndef JOINTS_STATE_H
 #define JOINTS_STATE_H

 #define PI 3.14159265359
 #include <cstdint>

 //INTEGRAL TERM IS USE BACKWARD EULER RULE
 typedef struct{
  int cntprev;    // Count per revolutions of encoder
  int rate;       // Controller loop rate (Hz)
  int diffcnt;     // Diff counter values
  int64_t total_cnt;  // Total counter values
  float rad;      // Current position in rads
  float radps;    // Current velocity in rads / secs
 } joints_state;

  void js_init(joints_state* js, int cnt_per_rev, int rate);
  void update_joints_state(joints_state* js);

  #endif /* JOINTS_STATE_H */