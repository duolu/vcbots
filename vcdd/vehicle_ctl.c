#include "vehicle_ctl.h"

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "errno.h"

#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <pthread.h>

#include "serial.h"
#include "sprotocol.h"
#include "joystick.h"


// ----------- configuration -------------------

int period = 100 * 1000; // 100 ms


struct vehicle_ctrl v_ctrl = {

		.mode = 0,

		.js_x = 0,
		.js_y = 0,

		.pwm_left = 0,
		.pwm_right = 0,
};

int stop_serial_tx = 0;
int stop_serial_rx = 0;
int stop_js = 0;





// ----------------------- main -----------------------------

int main() {

	int ret = 0;

	int sfd = serial_init("/dev/ttyACM0", 115200);
	if (sfd < 0) {
		printf("Unable to open serial port!!!\n");
		exit(-1);
	}

	// CAUTION: When the serial port is opened, Arduino board is reset.
	// We must wait for a while before sending the first message so that the
	// Arduino board is correctly setup.
	sleep(5);

	pthread_t tx_serial;
	pthread_t rx_serial;
	pthread_t js;

	ret = pthread_create(&tx_serial, NULL, serial_send_thread, &sfd);
	if (ret != 0) {
		fprintf(stderr, "Error - pthread_create() return code: %d\n", ret);
		exit(EXIT_FAILURE);
	}
	ret = pthread_create(&rx_serial, NULL, serial_recv_thread, &sfd);
	if (ret != 0) {
		fprintf(stderr, "Error - pthread_create() return code: %d\n", ret);
		exit(EXIT_FAILURE);
	}

	ret = pthread_create(&js, NULL, joystick_thread, &sfd);
	if (ret != 0) {
		fprintf(stderr, "Error - pthread_create() return code: %d\n", ret);
		exit(EXIT_FAILURE);
	}



	pthread_join(tx_serial, NULL);
	pthread_join(rx_serial, NULL);
	pthread_join(js, NULL);

	return 0;
}
