
#include <stdlib.h>
#include "PIDController.h"


PIDController::PIDController(struct PIDControllerConfig conf) {

        _kp = conf.kp;
        _ki = conf.ki;
        _kd = conf.kd;

        _deadzone = conf.deadzone;
        _output_max = conf.output_max;
        _output_min = conf.output_min;

        _period = conf.period;
        _output_factor = conf.output_factor;

        resetState();
}

void PIDController::setupGain(float kp, float ki, float kd) {

        _kp = kp;
        _ki = ki;
        _kd = kd;
        
}

void PIDController::setupDeadzone(float deadzone) {

        _deadzone = deadzone;
}

void PIDController::resetState() {

        out_p = 0;
        val_p = 0;
        err_p = 0;
        err_pp = 0;
}

float PIDController::control(float val_current, float val_dsr, float *up, float *ui, float *ud, float *u_raw) {

        err = val_dsr - val_current;
        if (err < _deadzone && err > -_deadzone) {

                out = out_p;
                
        } else {

                out = ctrl_pid_inc(out_p, err, err_p, err_pp, _kp, _ki, _kd, up, ui, ud, _period / 1000);
                
        }

        if (u_raw != NULL)
                *u_raw = out;

        out_p = out;
        out_saturated = out;
        if (out > _output_max)
                out_saturated = _output_max;
        if (out < _output_min)
                out_saturated = _output_min;
        
        
        return out_saturated;

        
}

// PID controller
static float PIDController::ctrl_pid(float err, float err_sum, float err_p,
               float kp, float ki, float kd, 
               float *up_out, float *ui_out, float *ud_out,
               float ts) {

  float u = 0;
  float up = kp * err;
  float ui = ki * ts * err_sum;
  float ud = kd * (err - err_p) / ts;

  u = up + ui + ud;

  if(up_out != NULL)
    *up_out = up;
  if(ui_out != NULL)
    *ui_out = ui;
  if(ud_out != NULL)
    *ud_out = ud;

  return u;
}

// PID controller Incremental Style
static float PIDController::ctrl_pid_inc(float u_p, 
                 float err, float err_p, float err_pp,
                 float kp, float ki, float kd, 
                 float *up_out, float *ui_out, float *ud_out,
                 float ts) {

  float up = kp * (err - err_p);
  float ui = ki * ts * err;
  float ud = kd * ((err - err_p) - (err_p - err_pp)) / ts;

  float delta_u = (up + ui + ud);
  
  float u = u_p + delta_u;

  if(up_out != NULL)
    *up_out = up;
  if(ui_out != NULL)
    *ui_out = ui;
  if(ud_out != NULL)
    *ud_out = ud;

  return u;
}







