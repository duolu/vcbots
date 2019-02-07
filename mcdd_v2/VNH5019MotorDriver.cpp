


#include "VNH5019MotorDriver.h"

VNH5019MotorDriver::VNH5019MotorDriver(struct VNH5019MotorDriverConfig conf) {

        _pwm_pin = conf.pwm_pin;
        _dir_a = conf.dir_a;
        _dir_b = conf.dir_b;
        _cs_pin = conf.cs_pin;

        _gear_ratio = conf.gear_ratio;

        pinMode(_pwm_pin,OUTPUT);
        pinMode(_dir_a,OUTPUT);
        pinMode(_dir_b,OUTPUT);
        pinMode(_cs_pin,INPUT);


      // Timer configuration
      // prescaler: clockI/O / 1
      // outputs enabled
      // phase-correct PWM
      // top of 400
      //
      // PWM frequency calculation
      // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
      
//        if (pwm_pin == 6 || pwm_pin == 7 || pwm_pin == 8) {
//                
//                TCCR4A = 0b10100000;
//                TCCR4B = 0b00010001;
//                ICR4 = 400;
//                
//        } else if (pwm_pin == 44 || pwm_pin == 45 || pwm_pin == 46) {
//                
//                TCCR5A = 0b10100000;
//                TCCR5B = 0b00010001;
//                ICR5 = 400;
//                
//        }
        
}

void VNH5019MotorDriver::setpwm(uint8_t pwm, int8_t dir) {

        if (dir > 0) {

                digitalWrite(_dir_a, HIGH);
                digitalWrite(_dir_b, LOW);

                analogWrite(_pwm_pin, pwm);
                
        } else if (dir < 0) {

                digitalWrite(_dir_a, LOW);
                digitalWrite(_dir_b, HIGH);

                analogWrite(_pwm_pin, pwm);
                
        } else {

                brake();
        }
        
        
}

uint32_t VNH5019MotorDriver::senseCurrent() {

        // current is in mA, 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
        (uint32_t)analogRead(_cs_pin) * 34;
}


void VNH5019MotorDriver::brake() {
        
        digitalWrite(_dir_a, LOW);
        digitalWrite(_dir_b, LOW);

        analogWrite(_pwm_pin, 0);
}

