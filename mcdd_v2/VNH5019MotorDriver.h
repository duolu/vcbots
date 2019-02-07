
#ifndef _VNH5019_MOTOR_DRIVER_H_
#define _VNH5019_MOTOR_DRIVER_H_


#include "Arduino.h"

struct VNH5019MotorDriverConfig {

        uint8_t pwm_pin;
        uint8_t dir_a;
        uint8_t dir_b;
        uint8_t cs_pin;

        float gear_ratio;
};

class VNH5019MotorDriver
{
public:

	VNH5019MotorDriver(struct VNH5019MotorDriverConfig conf);

        void setpwm(uint8_t pwm, int8_t dir);
        uint32_t senseCurrent();

        void brake();

        inline float getGearRatio() {

                return _gear_ratio;
        }

private:

        uint8_t _pwm_pin;
        uint8_t _dir_a;
        uint8_t _dir_b;
        uint8_t _cs_pin;

        float _gear_ratio;

};




#endif // _VNH5019_MOTOR_DRIVER_H_
