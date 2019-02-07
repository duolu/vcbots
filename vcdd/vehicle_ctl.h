/*
 * head_neck_ctl.h
 *
 *  Created on: Jun 14, 2018
 *      Author: priori
 */

#ifndef VEHICLE_CTL_H_
#define VEHICLE_CTL_H_

#include <stdint.h>
#include <time.h>


struct vehicle_ctrl {

	volatile int mode;

	volatile int js_x;
	volatile int js_y;

	volatile int pwm_left;
	volatile int pwm_right;

	volatile float omega_left_dsr;
	volatile float omega_right_dsr;

	volatile int stop_flag;
	volatile int start_open_loop_flag;
	volatile int start_close_loop_flag;

};

extern struct vehicle_ctrl v_ctrl;

static inline uint64_t get_us() {

	struct timespec spec;

	clock_gettime(CLOCK_MONOTONIC, &spec);

	uint64_t s = spec.tv_sec;
	uint64_t us = spec.tv_nsec / 1000 + s * 1000 * 1000;

	return us;
}


#endif /* VEHICLE_CTL_H_ */
