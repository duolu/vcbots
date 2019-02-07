
#include "PowertrainDDVNH5019.h"

PowertrainDDVNH5019::PowertrainDDVNH5019(struct PowertrainDDConfig conf) {

        _pwm_max = conf.pwm_max;
        _pwm_min = conf.pwm_min;
        _omega_left_max = conf.omega_left_max;
        _omega_right_max = conf.omega_right_max;
        _control_loop_period = conf.control_loop_period;


        _conf = conf;

        for (int i = 0; i < NR_MOTORS; i++) {

                _encoders[i] = conf.encoders[i];
                _controllers[i] = conf.controllers[i];
                _motors[i] = conf.motors[i];
        }

        mode = 0;

        state.omega_left_dsr = 0;
        state.omega_right_dsr = 0;

        for (int i = 0; i < NR_MOTORS; i++) {

                state.omega[i] = 0;
                state.counters[i] = 0;
                state.electric_current[i] = 0;

                state.pwm[i] = 0;
                state.dir[i] = 0;

                state.pwm_out[i] = 0;
                state.pwm_raw[i] = 0;
                state.pwm_up[i] = 0;
                state.pwm_ui[i] = 0;
                state.pwm_ud[i] = 0;
        }
        
}

void PowertrainDDVNH5019::setupControllerParameters(float kp, float ki, float kd) {

        for (int i = 0; i < NR_MOTORS; i++) {

                _controllers[i]->setupGain(kp, ki, kd);
        }
}

void PowertrainDDVNH5019::turn_on_open_loop() {

        if (mode == 0)
                mode = 1;
}

void PowertrainDDVNH5019::turn_on_close_loop() {

        if (mode == 0)
                mode = 2;

}

void PowertrainDDVNH5019::turn_off() {

        mode = 0;
        brake();
        reset();
}

void PowertrainDDVNH5019::reset() {

        state.omega_left_dsr = 0;
        state.omega_right_dsr = 0;

        for (int i = 0; i < NR_MOTORS; i++) {

                state.pwm[i] = 0;
                state.dir[i] = 0;

                state.pwm_out[i] = 0;
                state.pwm_raw[i] = 0;
                state.pwm_up[i] = 0;
                state.pwm_ui[i] = 0;
                state.pwm_ud[i] = 0;
        }
}

void PowertrainDDVNH5019::drive_open_loop(int16_t pwm_left, int16_t pwm_right) {

        if (mode == 0 || mode == 1) {

                if (pwm_left > _pwm_max)
                        pwm_left = _pwm_max;
                if (pwm_left < _pwm_min)
                        pwm_left = _pwm_min;
                if (pwm_right > _pwm_max)
                        pwm_right = _pwm_max;
                if (pwm_right < _pwm_min)
                        pwm_right = _pwm_min;

                for (int i = 0; i < NR_MOTORS; i++) {
        
                        if (i % 2 == 0) {

                                if (pwm_left > 0) {
                                
                                        state.pwm[i] = pwm_left;
                                        state.dir[i] = +1;
                                }
                                else {
                                
                                        state.pwm[i] = -pwm_left;
                                        state.dir[i] = -1;
                                }
                                
                        } else {
                                
                                if (pwm_right > 0) {
                                
                                        state.pwm[i] = pwm_right;
                                        state.dir[i] = +1;
                                }
                                else {
                                
                                        state.pwm[i] = -pwm_right;
                                        state.dir[i] = -1;
                                }
                                
                        }
                }
        }
}

void PowertrainDDVNH5019::drive_close_loop(float omega_left_dsr, float omega_right_dsr) {

        if (mode == 0 || mode == 2) {

                if (omega_left_dsr > _omega_left_max)
                        omega_left_dsr = _omega_left_max;
                if (omega_right_dsr > _omega_right_max)
                        omega_right_dsr = _omega_right_max;
        
                state.omega_left_dsr = omega_left_dsr;
                state.omega_right_dsr = omega_right_dsr;

        }
}

void PowertrainDDVNH5019::brake() {

        for (int i = 0; i < NR_MOTORS; i++) {

                _motors[i]->brake();
        }        
}

int PowertrainDDVNH5019::getMode() {

        return mode;
}

struct PowertrainDDState *PowertrainDDVNH5019::getCurrentState() {

        return &state;
}

void PowertrainDDVNH5019::run() {

        // sense
        for (int i = 0; i < NR_MOTORS; i++) {

                state.counters[i] = _encoders[i]->read();
                state.omega[i] = state.counters[i] / _encoders[i]->getCPT() / _motors[i]->getGearRatio() * 2 * PI / (_control_loop_period / 1000.0);
                state.electric_current[i] = _motors[i]->senseCurrent();

                // TODO: sense motor voltage
                
        }


        // control (only if in close loop mode)
        if (mode == 2) {
                for (int i = 0; i < NR_MOTORS; i++) {
                        
                        float omega_dsr = (i % 2 == 0) ? state.omega_left_dsr : state.omega_right_dsr;
                        state.pwm_out[i] = _controllers[i]->control(state.omega[i], omega_dsr, 
                                &state.pwm_up[i], &state.pwm_ui[i], &state.pwm_ud[i], &state.pwm_raw[i]);

                        state.pwm[i] = (state.pwm_out[i] > 0) ? (uint8_t)state.pwm_out[i] : -(uint8_t)state.pwm_out[i];
                        state.dir[i] = (state.pwm_out[i] > 0) ? +1 : -1;
                }
        }


        // actuate (only if powertrain is on)
        if (mode > 0) {
                for (int i = 0; i < NR_MOTORS; i++) {
        
                        _motors[i]->setpwm(state.pwm[i], state.dir[i]);
                }
        }
}








