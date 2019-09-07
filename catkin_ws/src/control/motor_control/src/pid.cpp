#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include <cassert>
#include "pid/pid.h"
extern bool DEBUG;

using namespace std;
using namespace PID;

pid::pid():
  _max_status(false),
  _min_status(false),
  _kp_status(false),
  _ki_status(false),
  _kd_status(false),
  _is_ready(false),
  _pre_error(0),
  _integral(0)
  {}

pid::pid(double max, double min):
  _max(max), 
  _min(min),
  _max_status(true),
  _min_status(true),
  _kp_status(false),
  _ki_status(false),
  _kd_status(false),
  _is_ready(false),
  _pre_error(0),
  _integral(0)
  {assert(max>min);} // Make sure max bigger than min

pid::pid(double max, double min, double kp, double kd, double ki):
  _max(max), 
  _min(min),
  _kp(kp),
  _ki(ki),
  _kd(kd),
  _max_status(true),
  _min_status(true),
  _kp_status(true),
  _ki_status(true),
  _kd_status(true),
  _is_ready(true),
  _pre_error(0),
  _integral(0)
  {assert(max>min);} // Make sure max bigger than min

void pid::resetPid(void){
 _pre_error = 0;
 _integral = 0;
}

double pid::calculate(double setpoint, double pv, double dt){
  _is_ready = (_max_status and _min_status and _kp_status and _ki_status and _kd_status);
  assert(_is_ready); // Check if controller ready
  // Calculate error
  double error = setpoint - pv;
  // Propotional term
  double Pout = error * _kp;
  // Integral term
  _integral += error * dt;
  double Iout = _integral * _ki;
  // Derivative term
  double derivative;
  double Dout;
  if(dt!=0.0){
    derivative = (error - _pre_error) / dt;
    Dout = derivative * _kd;
  } else {derivative = 0.0, Dout = 0.0;}
  // Calculate total ouput
  double output = Pout + Iout + Dout;
  if(DEBUG) printf("[PID] error: %f, integral: %f, derivative: %f, out: %f\n", error, _integral, derivative, output);
  // Restrict to in range [_min, _max]
  if(output > _max) output = _max;
  else if(output < _min) output = _min;
  // Save error to previous error
  _pre_error = error;
  
  return output;
}
#endif
