

#ifndef _SERIAL_API_H_
#define _SERIAL_API_H_


#define SERIAL_STATE_INIT      0
#define SERIAL_STATE_MAGIC1    1
#define SERIAL_STATE_MAGIC2    2
#define SERIAL_STATE_PROTO     3

#define SERIAL_MAGIC_1 'A'
#define SERIAL_MAGIC_2 'F'


// ---------------------------- OPCODE in the protocol -----------------------


// high-to-low

#define OPCODE_OPEN_LOOP                0x00

#define OPCODE_CLOSE_LOOP               0x10

//#define OPCODE_PAN_TILT                 0x20

#define OPCODE_SETUP                    0x60


#define OPCODE_STOP                     0x70
#define OPCODE_START_OPEN_LOOP          0x71
#define OPCODE_START_CLOSE_LOOP         0x72


// low-to-high

#define OPCODE_VEHICLE_STATE_2WD         0x80
#define OPCODE_VEHICLE_STATE_4WD         0x81
#define OPCODE_VEHICLE_STATE_6WD         0x82

#define OPCODE_CONTROLLER_STATE_2WD            0x90
#define OPCODE_CONTROLLER_STATE_4WD            0x91
#define OPCODE_CONTROLLER_STATE_6WD            0x92

struct h2l_header {

        uint8_t magic1;
        uint8_t magic2;
        uint8_t len;
        uint8_t opcode;
};

// ---------------------------- high-to-low message -----------------------

struct open_loop_msg {

        int16_t pwm_left;
        int16_t pwm_right;
};

struct close_loop_msg {

        float omega_left_dsr;
        float omega_right_dsr;
};

struct setup_msg {

        float kp;
        float ki;
        float kd;
        float deadzone;
};


// the start message is header only
struct start_msg {
        
};

// the stop message is header only
struct stop_msg {
        
};

// ---------------------------- low-to-high message -----------------------

struct power_state_msg {

        float power_voltage;
        float motor_voltage;
};

struct motor_state_msg {

        float           omega;
        int16_t         pwm; // CAUTION: the sign of this pwm value indicate the direction
        uint16_t        encoder_count;
        uint32_t        current;
};

struct imu_state_msg {

        float acc_x;
        float acc_y;
        float acc_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float mag_x;
        float mag_y;
        float mag_z;
        float quaternion_w;
        float quaternion_x;
        float quaternion_y;
        float quaternion_z;
};

struct v_state_2WD_msg {

        uint32_t                timestamp;
        struct power_state_msg  p_state;
        struct imu_state_msg    imu_state;
        struct motor_state_msg  m_states[2];
        
};

struct v_state_4WD_msg {

        uint32_t                timestamp;
        struct power_state_msg  p_state;
        struct imu_state_msg    imu_state;
        struct motor_state_msg  m_states[4];
        
};

struct v_state_6WD_msg {

        uint32_t                timestamp;
        struct power_state_msg  p_state;
        struct imu_state_msg    imu_state;
        struct motor_state_msg  m_states[6];
        
};

struct controller_state_msg {

        float           err;
        float           out;
        float           up;
        float           ui;
        float           ud;
};

struct c_state_2WD_msg {

        float                   omega_left_dsr;
        float                   omega_right_dsr;
        struct controller_state_msg c_states[2];
};

struct c_state_4WD_msg {

       float                   omega_left_dsr;
       float                   omega_right_dsr;
       struct controller_state_msg c_states[4];
};

struct c_state_6WD_msg {

        float                   omega_left_dsr;
        float                   omega_right_dsr;
        struct controller_state_msg c_states[6];
};


typedef void (*handler_t)(uint8_t len, uint8_t *buf);


class SerialAPI {

public:

        SerialAPI();
        void runStateMachine();

        void sendVehicleState2WD(struct v_state_2WD_msg *msg);
        void sendVehicleState4WD(struct v_state_4WD_msg *msg);
        void sendVehicleState6WD(struct v_state_6WD_msg *msg);

        void sendControllerState2WD(struct c_state_2WD_msg *msg);
        void sendControllerState4WD(struct c_state_4WD_msg *msg);
        void sendControllerState6WD(struct c_state_6WD_msg *msg);

        void registerHandler(uint8_t opcode, handler_t handler);


private:

        int state = SERIAL_STATE_INIT;
        int len = 0;
        handler_t handlers[256];
        uint8_t recv_buf[256];


        // CAUTION: These are all blocking sending methods
        void sendByte(uint8_t b);
        void sendUInt16(uint16_t i);
        void sendUInt32(uint32_t l);
        void sendFloat32(float f);

        void sendHeader(byte len, byte opcode);
        void sendMsgBody(uint8_t *msg, unsigned int len);


        // CAUTION: These are all blocking receiving methods
        uint8_t recvByte();
        uint16_t recvUInt16();
        uint32_t recvUInt32();
        float recvFloat32();


        void dispatchCommand(uint8_t opcode, uint8_t len, uint8_t *buf);
        
};

















#endif // _SERIAL_API_H_
