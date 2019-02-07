

#ifndef _QUADRATURE_ENCODER_H_
#define _QUADRATURE_ENCODER_H_

#include "Encoder.h"


struct EncoderConfig {

        uint8_t pin_a;
        uint8_t pin_b;
        float count_per_turn;
};


// TODO: write a simplified version of quadrature encoder with interrupt support 

class QuadratureEncoder {

public:

        QuadratureEncoder(struct EncoderConfig conf);

        inline int32_t read() {

                encoder.read();
        }
        inline void write(int32_t p) {

                encoder.write(p);
        }

        inline float getCPT() {
                
                return _count_per_turn;
        }
private:

        Encoder encoder;

        uint8_t _pin_a;
        uint8_t _pin_b;
        float _count_per_turn;

};


#endif // _QUADRATURE_ENCODER_H_
