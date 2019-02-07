

#include "QuadratureEncoder.h"
#include "PIDController.h"
#include "VNH5019MotorDriver.h"
#include "PowertrainDDVNH5019.h"

#include "SerialAPI.h"

#include <TimerOne.h>


// ---------------------------- global configuration -----------------------

#define CONFIG_PLATFORM_VC_TRUCK_V2

const int16_t PWM_MAX = 255;
const int16_t PWM_MIN = -255;
const float OMEGA_MAX = 20;
const int CTRL_LOOP_PERIOD = 50;


// ---------------------------- hardware platform configuration ---------------


#if defined(CONFIG_PLATFORM_VC_TRUCK_V2)

// motor configuration

struct VNH5019MotorDriverConfig m_conf_left_front = {

        6,      // pwm
        30,     // direction a
        31,     // direction b
        2,      // current sensor
        26.851, // gear ratio
};

struct VNH5019MotorDriverConfig m_conf_right_front = {

        7,      // pwm
        32,     // direction a
        33,     // direction b
        3,      // current sensor
        26.851, // gear ratio
};

struct VNH5019MotorDriverConfig m_conf_left_middle = {

        8,      // pwm
        34,     // direction a
        35,     // direction b
        4,      // current sensor
        26.851, // gear ratio
};

struct VNH5019MotorDriverConfig m_conf_right_middle = {

        44,     // pwm
        36,     // direction a
        37,     // direction b
        5,      // current sensor
        26.851  // gear ratio
};

struct VNH5019MotorDriverConfig m_conf_left_rear = {

        45,     // pwm
        38,     // direction a
        39,     // direction b
        8,      // current sensor
        26.851, // gear ratio
};

struct VNH5019MotorDriverConfig m_conf_right_rear = {

        46,     // pwm
        40,     // direction a
        41,     // direction b
        9,      // current sensor
        26.851, // gear ratio
};




VNH5019MotorDriver mlf(m_conf_left_front);
VNH5019MotorDriver mrf(m_conf_right_front);
VNH5019MotorDriver mlm(m_conf_left_middle);
VNH5019MotorDriver mrm(m_conf_right_middle);
VNH5019MotorDriver mlr(m_conf_left_rear);
VNH5019MotorDriver mrr(m_conf_right_rear);


// encoder configuration


struct EncoderConfig e_conf_left_front = {
        
        2,              // pin_a;
        4,              // pin_b;
        48,             // cycle_per_turn;
};

struct EncoderConfig e_conf_right_front = {
        
        3,              // pin_a;
        5,              // pin_b;
        48,             // cycle_per_turn;
};

struct EncoderConfig e_conf_left_middle = {
        
        18,              // pin_a;
        22,              // pin_b;
        48,             // cycle_per_turn;
};

struct EncoderConfig e_conf_right_middle = {
        
        19,              // pin_a;
        23,              // pin_b;
        48,             // cycle_per_turn;
};

struct EncoderConfig e_conf_left_rear = {
        
        20,              // pin_a;
        24,              // pin_b;
        48,             // cycle_per_turn;
};

struct EncoderConfig e_conf_right_rear = {
        
        21,              // pin_a;
        25,              // pin_b;
        48,             // cycle_per_turn;
};




QuadratureEncoder elf(e_conf_left_front);
QuadratureEncoder erf(e_conf_right_front);
QuadratureEncoder elm(e_conf_left_middle);
QuadratureEncoder erm(e_conf_right_middle);
QuadratureEncoder elr(e_conf_left_rear);
QuadratureEncoder err(e_conf_right_rear);


// controller configuration

// NOTE: All motors share the same controller configuration
struct PIDControllerConfig c_conf {

        1.0,                    // kp
        1.0,                    // ki
        0.0,                    // kd
        
        0.0,                    // deadzone
        255,                    // output_max
        -255,                   // output_min

        CTRL_LOOP_PERIOD,       // period, in ms
};

PIDController clf(c_conf);
PIDController crf(c_conf);
PIDController clm(c_conf);
PIDController crm(c_conf);
PIDController clr(c_conf);
PIDController crr(c_conf);


// powertrain configuration

PowertrainDDConfig pt_conf = {
        
        {
                &elf,
                &erf,
                &elm,
                &erm,
                &elr,
                &err,
        },
        {
                &clf,
                &crf,
                &clm,
                &crm,
                &clr,
                &crr,
        },
        {
                &mlf,
                &mrf,
                &mlm,
                &mrm,
                &mlr,
                &mrr,
        },
        PWM_MAX,
        PWM_MIN,
        OMEGA_MAX,
        OMEGA_MAX,
        CTRL_LOOP_PERIOD,
};


PowertrainDDVNH5019 pt(pt_conf);


// serial API configuration

SerialAPI sapi;

struct v_state_6WD_msg v_state_msg;
struct c_state_6WD_msg c_state_msg;


// glue code between serial API and motion controller

void on_stop(uint8_t len, uint8_t *buf) {

        pt.turn_off();
}

void on_start_open_loop(uint8_t len, uint8_t *buf) {

        pt.turn_on_open_loop();
}

void on_start_close_loop(uint8_t len, uint8_t *buf) {

        pt.turn_on_close_loop();
}

void on_open_loop(uint8_t len, uint8_t *buf) {

        struct open_loop_msg *msg = (struct open_loop_msg *)buf;

        pt.drive_open_loop(msg->pwm_left, msg->pwm_right);
        
}

void on_close_loop(uint8_t len, uint8_t *buf) {

        struct close_loop_msg *msg = (struct close_loop_msg *)buf;

        pt.drive_close_loop(msg->omega_left_dsr, msg->omega_right_dsr);
        
}

void on_setup(uint8_t len, uint8_t *buf) {

        struct setup_msg *msg = (struct setup_msg *)buf;

        pt.setupControllerParameters(msg->kp, msg->ki, msg->kd);
        
}


void setup_platform() {

        sapi.registerHandler(OPCODE_STOP, on_stop);
        sapi.registerHandler(OPCODE_START_OPEN_LOOP, on_start_open_loop);
        sapi.registerHandler(OPCODE_START_CLOSE_LOOP, on_start_close_loop);

        sapi.registerHandler(OPCODE_OPEN_LOOP, on_open_loop);
        sapi.registerHandler(OPCODE_CLOSE_LOOP, on_close_loop);
        
        sapi.registerHandler(OPCODE_SETUP, on_setup);
        
}

void update_v_state_msg(struct v_state_6WD_msg *msg, uint32_t ts, uint32_t loop_count) {

        PowertrainDDState *pt_state = pt.getCurrentState();

        msg->timestamp = ts;

        msg->p_state.power_voltage = 123;
        msg->p_state.motor_voltage = 456;

        // TODO: update IMU state

        for (int i = 0; i < NR_MOTORS; i++) {

                msg->m_states[i].omega = pt_state->omega[i];
                msg->m_states[i].pwm = (int16_t)pt_state->pwm[i] * (int16_t)pt_state->dir[i];
                msg->m_states[i].encoder_count = pt_state->counters[i];
                msg->m_states[i].current = pt_state->electric_current[i];
        }
                
        
}

#endif // define (CONFIG_PLATFORM_VC_TRUCK_V2)



// ----------------------------------- interrupt -------------------------

// Timer interrupt counter. It is reset when new loop period comes.
int timer_counter = 0;

// Timer flag for the controller loop
volatile int timer_ctrl_loop_flag = 0;

// CAUTION: This function is called in interrupt!!! Must make it short!!!
void timer_update() {

  timer_counter++;

  if (timer_counter >= CTRL_LOOP_PERIOD) {
    timer_ctrl_loop_flag = 1;
    timer_counter = 0;
  }

}


// ----------------------------------- setup --------------------------

void setup() {

        Serial.begin(115200);

        setup_platform();

        Timer1.initialize(1000); // NOTE: period is in us
        Timer1.attachInterrupt(timer_update);

        pt.reset();
        pt.brake();
}

// ----------------------------------- loop --------------------------

unsigned long loop_count = 0;

void loop() {

        if (timer_ctrl_loop_flag > 0) {

                uint32_t ts = millis();

                pt.run();

                update_v_state_msg(&v_state_msg, ts, loop_count);
                sapi.sendVehicleState6WD(&v_state_msg);

                loop_count++;
                
                timer_ctrl_loop_flag = 0;

//                Serial.print(ts);
//                Serial.print('\t');
//                Serial.print(loop_count);
//                Serial.print('\n');
        }

        // Run Serial State Machine
        // CAUTION: Never block or delay or spend too much time here
        sapi.runStateMachine();
        

}









