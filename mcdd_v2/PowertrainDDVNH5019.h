
#ifndef _POWERTRAIN_DD_H_
#define _POWERTRAIN_DD_H_


#include "VNH5019MotorDriver.h"
#include "QuadratureEncoder.h"
#include "PIDController.h"

//#define NR_MOTORS       2
//#define NR_MOTORS       3
#define NR_MOTORS       6




/*
 * Mapping between motor ID and physical configuration:
 * 
 * 2WD:
 * 
 * motor 0 ---> left wheel
 * motor 1 ---> right wheel
 * 
 * 4WD:
 * 
 * motor 0 ---> left front wheel
 * motor 1 ---> right front wheel
 * motor 2 ---> left rear wheel
 * motor 3 ---> right rear wheel
 * 
 * 6WD:
 * 
 * motor 0 ---> left front wheel
 * motor 1 ---> right front wheel
 * motor 2 ---> left middle wheel
 * motor 3 ---> right middle wheel
 * motor 4 ---> left rear wheel
 * motor 5 ---> right rear wheel
 * 
 */
struct PowertrainDDConfig {

        QuadratureEncoder       *encoders[NR_MOTORS];
        PIDController           *controllers[NR_MOTORS];
        VNH5019MotorDriver      *motors[NR_MOTORS];

        int16_t pwm_max;
        int16_t pwm_min;
        float omega_left_max;
        float omega_right_max;
        float control_loop_period;      // in ms
};

struct PowertrainDDState {

        // desired state
        float omega_left_dsr;
        float omega_right_dsr;

        // current state
        float omega[NR_MOTORS];         // current motor speed
        uint16_t counters[NR_MOTORS];      // encoder counters since last iteration
        uint16_t electric_current[NR_MOTORS];    // electric current of each motor

        // output
        uint8_t pwm[NR_MOTORS];
        int8_t dir[NR_MOTORS];

        // debug state
        float pwm_out[NR_MOTORS]; // controller output after threashold
        float pwm_raw[NR_MOTORS]; // raw output of the controller without threashold
        float pwm_up[NR_MOTORS];
        float pwm_ui[NR_MOTORS];
        float pwm_ud[NR_MOTORS];

};

class PowertrainDDVNH5019 {

public:

        PowertrainDDVNH5019(struct PowertrainDDConfig conf);

        void setupControllerParameters(float kp, float ki, float kd);

        void turn_on_open_loop();
        void turn_on_close_loop();
        void turn_off();
        void reset();

        void drive_open_loop(int16_t pwm_left, int16_t pwm_right);
        void drive_close_loop(float omega_left_dsr, float omega_right_dsr);
        void brake();

        int getMode();
        struct PowertrainDDState *getCurrentState();


        // This method must be called in the main loop
        void run();


private:

        int mode; // 0 - off, 1 - open loop, 2 - close loop

        struct PowertrainDDConfig _conf;

        int16_t _pwm_max;
        int16_t _pwm_min;
        float _omega_left_max;
        float _omega_right_max;
        float _control_loop_period;      // in ms
        int _nr_motors;

        // actuator, sensor, and controller
        QuadratureEncoder       *_encoders[NR_MOTORS];
        PIDController           *_controllers[NR_MOTORS];
        VNH5019MotorDriver      *_motors[NR_MOTORS];

        struct PowertrainDDState state;

};


















#endif // _POWERTRAIN_DD_H_
