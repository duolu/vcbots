/*
 * sprotocol.c
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

#include "serial.h"
#include "sprotocol.h"
#include "vehicle_ctl.h"

extern int period;
extern int stop_serial_tx;
extern int stop_serial_rx;


// ---------------- serial -------------------

static int h2l_send_open_loop_msg(int sfd, int pwm_left, int pwm_right) {

	int ret = 0;
	struct open_loop_msg msg;

	h2l_set_header(&msg.header, sizeof(msg) - sizeof(msg.header),
			OPCODE_OPEN_LOOP);

	msg.pwm_left = (int16_t)pwm_left;
	msg.pwm_right = (int16_t)pwm_right;

	ret = serial_send_n_bytes(sfd, (char *) &msg, sizeof(msg));
	if (ret < 0) {

		printf("Unable to write to serial port, ret=%d, errno=%s.\n", ret, strerror(errno));
	}
	//serial_flush(sfd);

	return ret;
}

static int h2l_send_header_only_msg(int sfd, uint8_t opcode) {

	int ret = 0;
	struct h2l_header msg;

	h2l_set_header(&msg, 0, opcode);

	ret = serial_send_n_bytes(sfd, (char *) &msg, sizeof(msg));
	if (ret < 0) {

		printf("Unable to write to serial port, ret=%d, errno=%s.\n", ret, strerror(errno));
	}
	//serial_flush(sfd);

	return ret;
}


static int h2l_send_stop_msg(int sfd) {

	return h2l_send_header_only_msg(sfd, OPCODE_STOP);
}

static int h2l_send_start_open_loop_msg(int sfd) {

	return h2l_send_header_only_msg(sfd, OPCODE_START_OPEN_LOOP);
}

static int h2l_send_start_close_loop_msg(int sfd) {

	return h2l_send_header_only_msg(sfd, OPCODE_START_CLOSE_LOOP);
}

void *serial_send_thread(void *para) {

	// CAUTION: always sleep 1ms as an small interval
	struct timespec idle;
	idle.tv_sec = 0;
	idle.tv_nsec = 1000 * 1000;

	int sfd = *((int *) para);

	printf("serial sending thread started.\n");

	uint64_t init_ts = get_us();

	// serial loop
	for (uint32_t i = 0; !stop_serial_tx; i++) {


		if (v_ctrl.mode == 1) {

			v_ctrl.pwm_left = (v_ctrl.js_y + v_ctrl.js_x) * 50 / 65536;
			v_ctrl.pwm_right = (v_ctrl.js_y - v_ctrl.js_x) * 50 / 65536;


			h2l_send_open_loop_msg(sfd, v_ctrl.pwm_left, v_ctrl.pwm_right);

			printf("[tx] open_loop: pwm_left=%d, pwm_right=%d\n", v_ctrl.pwm_left, v_ctrl.pwm_right);

		} else if (v_ctrl.mode == 2) {

			// TODO:

		} else {

			// Nothing to do.


		}

		if (v_ctrl.start_open_loop_flag > 0) {
			v_ctrl.start_open_loop_flag = 0;

			h2l_send_start_open_loop_msg(sfd);
		}

		if (v_ctrl.start_close_loop_flag > 0) {
			v_ctrl.start_close_loop_flag = 0;

			h2l_send_start_close_loop_msg(sfd);
		}

		if (v_ctrl.stop_flag > 0) {
			v_ctrl.stop_flag = 0;

			h2l_send_stop_msg(sfd);
		}

//		printf(
//				"[tx] pan_left=%d, tilt_left=%d, pan_right=%d, tilt_right=%d\n",
//				pt_ctrl.pan_left, pt_ctrl.tilt_left, pt_ctrl.pan_right,
//				pt_ctrl.tilt_right);

		uint64_t next_ts = init_ts + period * (i + 1);
		uint64_t ts = get_us();

//		printf("[serial] next_ts=%lu, ts=%lu\n", next_ts - init_ts, ts - init_ts);

		while (ts < next_ts) {

			nanosleep(&idle, NULL);
			ts = get_us();
		}

	}

	printf("serial sending thread exited.\n");

	return NULL;
}

static int h2l_recv_process(int sfd, char serial_buff[]) {

	int ret;
	int i;

	uint8_t len = (uint8_t)serial_buff[2];
	uint8_t opcode = (uint8_t)serial_buff[3];

	// receive payload
	ret = serial_recv_n_bytes(sfd, serial_buff + sizeof(struct h2l_header), len);

//	printf("[rx]");
//	for(i = 0; i < len + sizeof(struct h2l_header); i++) {
//		printf("%02X ", serial_buff[i] & 0xFF);
//	}
//	printf("\n");

	switch (opcode) {

	case OPCODE_VEHICLE_STATE_6WD: {

		struct v_state_6WD_msg *msg = (struct v_state_6WD_msg *) (serial_buff);

//		printf("[v_state] ts=%u, pv=%.2f, mv=%.2f, "
//				"omega[0]=%.2f, pwm[0]=%d, count[0]=%u, current[0]=%u, "
//				"omega[1]=%.2f, pwm[1]=%d, count[1]=%u, current[1]=%u\n",
//				msg->timestamp, msg->p_state.power_voltage, msg->p_state.motor_voltage,
//				msg->m_states[0].omega, msg->m_states[0].pwm, msg->m_states[0].encoder_count, msg->m_states[0].current,
//				msg->m_states[1].omega, msg->m_states[1].pwm, msg->m_states[1].encoder_count, msg->m_states[1].current);


//		printf("[rx] ts=%u, pan_left=%d, tilt_left=%d, "
//				"pan_right=%d, tilt_right=%d, "
//				"pan_neck=%d, tilt_neck=%d\n",
//				msg->timestamp, msg->pan_left_pos, msg->tilt_left_pos,
//				msg->pan_right_pos, msg->tilt_right_pos,
//				msg->pan_neck_pos, msg->tilt_neck_pos);


		break;
	}
	default: break;
	}

	return 0;
}

void *serial_recv_thread(void *para) {

	int ret;
	int sfd = *((int *) para);
	char c;
	int comm_state = SERIAL_STATE_INIT;
	char serial_buff[128];

	printf("serial receiving thread started.\n");

	while (!stop_serial_rx) {

		// CAUTION: This is polling every 1 ms!!!
		// receive protocol header
		ret = serial_recv_byte(sfd, &c);
		if(ret < 0) {
			printf("Serial read error!!!\n");
			break;
		} else if(ret == 0) {

			printf("Nothing is received!!!\n");
			usleep(1000 * 1000);
			continue;
		}


		serial_buff[comm_state] = c;

		//printf("%02X ", (c & 0xFF));

		switch (comm_state) {
		case SERIAL_STATE_INIT: {
			if (c == SERIAL_MAGIC_1)
				comm_state = SERIAL_STATE_MAGIC1;
			else
				comm_state = SERIAL_STATE_INIT;
			break;
		}
		case SERIAL_STATE_MAGIC1: {
			if (c == SERIAL_MAGIC_2)
				comm_state = SERIAL_STATE_MAGIC2;
			else
				comm_state = SERIAL_STATE_INIT;
			break;
		}
		case SERIAL_STATE_MAGIC2: {
			comm_state = SERIAL_STATE_PROTO;
			break;
		}
		case SERIAL_STATE_PROTO: {

			h2l_recv_process(sfd, serial_buff);

			comm_state = SERIAL_STATE_INIT;
			break;
		}
		default: {
			comm_state = SERIAL_STATE_INIT;
			break;
		}

		}

	}

	printf("serial receiving thread exited.\n");

	return NULL;
}

