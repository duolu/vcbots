

#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

struct PIDControllerConfig {

        float kp;
        float ki;
        float kd;

        float deadzone;
        float output_max;
        float output_min;

        float period; // in ms
        float output_factor;
};

class PIDController {

public:

        PIDController(struct PIDControllerConfig);
        void setupGain(float kp, float ki, float kd);
        void setupDeadzone(float deadzone);
        void resetState();

        float control(float val_current, float out_dsr, float *up, float *ui, float *ud, float *u_raw);

        static float ctrl_pid(float err, float err_sum, float err_p,
                float kp, float ki, float kd, 
                float *up_out, float *ui_out, float *ud_out,
                float ts);
        static float ctrl_pid_inc(float u_p, 
                float err, float err_p, float err_pp,
                float kp, float ki, float kd, 
                float *up_out, float *ui_out, float *ud_out,
                float ts);
private:

        float _kp;
        float _ki;
        float _kd;

        float _deadzone;
        float _output_max;
        float _output_min;

        float _period;
        float _output_factor;

        float out_saturated;
        float out;
        float out_p;
        float val_p;
        float err;
        float err_p;
        float err_pp;
        
};












#endif // _PID_CONTROLLER_H_
