
#include "QuadratureEncoder.h"

QuadratureEncoder::QuadratureEncoder(struct EncoderConfig conf) :
        encoder(conf.pin_a, conf.pin_b)
{

        _pin_a = conf.pin_a;
        _pin_b = conf.pin_b;
        _count_per_turn = conf.count_per_turn;
}

