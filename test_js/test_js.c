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

// ---------------- joystick -------------------

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

struct js_event {

	uint32_t time; /* event timestamp in milliseconds */
	int16_t value; /* value */
	uint8_t type; /* event type */
	uint8_t number; /* axis/button number */
};


void process_js_event(struct js_event e) {

	uint8_t type = e.type & ~JS_EVENT_INIT;

	// only response when the button is released
	if (type == JS_EVENT_BUTTON && e.value == 1) {

		printf("Butten event: ts=%u, number=%u, value=%d\n", e.time, e.number,
				e.value);

		switch (e.number) {

		case 0:

			break;
		case 1:

			break;
		case 2:

			break;
		case 3:

			break;
		case 4:
		case 5:
		case 6:
		case 7:

			break;
		default:

			break;
		}

	} else if (type == JS_EVENT_AXIS) {

		printf("AXIS event: ts=%u, number=%u, value=%d\n", e.time, e.number,
				e.value);

		switch (e.number) {

		case 0:
			break;
		case 1:
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

int main() {

	int ret = 0;

	int fd = open("/dev/input/js0", O_RDONLY, O_NONBLOCK);
	if (fd < 0) {
		printf("Cannot find a joystick attached.\n");
		exit(0);
	}

	while (1) {

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

