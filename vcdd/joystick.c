/*
 * joystick.c
 *
 *  Created on: Jun 14, 2018
 *      Author: priori
 */

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "errno.h"

#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>

#include "joystick.h"
#include "vehicle_ctl.h"

extern int stop_js;

void process_js_event(struct js_event e) {

	uint8_t type = e.type & ~JS_EVENT_INIT;

	// only response when the button is released
	if (type == JS_EVENT_BUTTON && e.value == 1) {

//		printf("Butten event: ts=%u, number=%u, value=%d\n", e.time, e.number,
//				e.value);

		switch (e.number) {

		case 0:
			v_ctrl.mode = 1;
			v_ctrl.start_open_loop_flag = 1;

			printf("[joystick] start open loop\n");
			break;
		case 1:
			v_ctrl.mode = 2;
			v_ctrl.start_close_loop_flag = 1;

			printf("[joystick] start open loop\n");
			break;
		case 2:

			break;
		case 3:
			v_ctrl.mode = 0;
			v_ctrl.stop_flag = 1;

			printf("[joystick] stop\n");
			break;
		case 4:
		case 5:
		case 6:
		case 7:

			break;
		case 8:

			break;
		case 9:

			break;
		default:

			break;
		}

	} else if (type == JS_EVENT_AXIS) {

//		printf("AXIS event: ts=%u, number=%u, value=%d\n", e.time, e.number,
//				e.value);

		switch (e.number) {

		case 0:
			v_ctrl.js_x = e.value;
			break;
		case 1:
			v_ctrl.js_y = -e.value;
			break;
		case 2:

			break;
		case 3:

			break;
		case 4:

			break;
		case 5:

			break;
		default:

			break;
		}
	}

}

void *joystick_thread(void *para) {

	int ret = 0;

	int fd = open("/dev/input/js0", O_RDONLY, O_NONBLOCK);
	if (fd < 0) {
		printf("Cannot find a joystick attached.\n");
		exit(0);
	}

	while (!stop_js) {

		struct js_event e;
		ret = read(fd, &e, sizeof(e));

		if (ret < 0) {

			printf("Unable to read joystick event\n");
			break;
		}

		process_js_event(e);

	}

	return NULL;
}

