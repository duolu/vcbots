
#include "Arduino.h"
#include "SerialAPI.h"


SerialAPI::SerialAPI() {

        state = SERIAL_STATE_INIT;
        len = 0;

        int nr_handlers = sizeof(handlers) / sizeof(handler_t);
        for (int i = 0; i < nr_handlers; i++) {

                handlers[i] = NULL;
        }

        


}

void SerialAPI::runStateMachine() {

        uint8_t c;

        if (Serial.available() > 0) {

                c = Serial.read();
                
        } else {

                return;
        }

        //  Serial.print(c, HEX);
        //  Serial.print(' ');

        switch (state) {
        case SERIAL_STATE_INIT: {
                if (c == SERIAL_MAGIC_1)
                        state = SERIAL_STATE_MAGIC1;
                else
                        state = SERIAL_STATE_INIT;
                break;
        }
        case SERIAL_STATE_MAGIC1: {
                if (c == SERIAL_MAGIC_2)
                        state = SERIAL_STATE_MAGIC2;
                else
                        state = SERIAL_STATE_INIT;
                break;
        }
        case SERIAL_STATE_MAGIC2: {
                len = c;
                state = SERIAL_STATE_PROTO;
                break;
        }
        case SERIAL_STATE_PROTO: {

                // TODO: limit the length of command message here
                
                for (int i = 0; i < len; i++) {

                        recv_buf[i] = recvByte();
                }
                dispatchCommand(c, len, recv_buf);

                len = 0;
                state = SERIAL_STATE_INIT;
                break;
        }
        default: {
                state = SERIAL_STATE_INIT;
                break;
        }

        }
}

void SerialAPI::sendVehicleState2WD(struct v_state_2WD_msg *msg) {

        uint8_t len = sizeof(struct v_state_2WD_msg);
        sendHeader(len, OPCODE_VEHICLE_STATE_2WD);
        sendMsgBody((uint8_t *)msg, len);
}
void SerialAPI::sendVehicleState4WD(struct v_state_4WD_msg *msg) {

        uint8_t len = sizeof(struct v_state_4WD_msg);
        sendHeader(len, OPCODE_VEHICLE_STATE_4WD);
        sendMsgBody((uint8_t *)msg, len);
}
void SerialAPI::sendVehicleState6WD(struct v_state_6WD_msg *msg) {

        uint8_t len = sizeof(struct v_state_6WD_msg);
        sendHeader(len, OPCODE_VEHICLE_STATE_6WD);
        sendMsgBody((uint8_t *)msg, len);
}

void SerialAPI::sendControllerState2WD(struct c_state_2WD_msg *msg) {

        uint8_t len = sizeof(struct c_state_2WD_msg);
        sendHeader(len, OPCODE_CONTROLLER_STATE_2WD);
        sendMsgBody((uint8_t *)&msg, len);
}

void SerialAPI::sendControllerState4WD(struct c_state_4WD_msg *msg) {

        uint8_t len = sizeof(struct c_state_4WD_msg);
        sendHeader(len, OPCODE_CONTROLLER_STATE_4WD);
        sendMsgBody((uint8_t *)&msg, len);
}

void SerialAPI::sendControllerState6WD(struct c_state_6WD_msg *msg) {

        uint8_t len = sizeof(struct c_state_6WD_msg);
        sendHeader(len, OPCODE_CONTROLLER_STATE_6WD);
        sendMsgBody((uint8_t *)&msg, len);
}

void SerialAPI::registerHandler(uint8_t opcode, handler_t handler) {

        handlers[opcode] = handler;
}






void SerialAPI::sendByte(uint8_t b) {

        while (Serial.write(b) <= 0)
                ;

}

void SerialAPI::sendUInt16(uint16_t i) {
        
        byte *cp = (byte *) &i;
        
        byte c0 = cp[0];
        byte c1 = cp[1];
        
        sendByte(c0);
        sendByte(c1);
        Serial.flush();

}

void SerialAPI::sendUInt32(uint32_t l) {
        
        byte *cp = (byte *) &l;
        
        //  byte c0 = (byte)(l & 0x000000FF);
        //  byte c1 = (byte)((l & 0x0000FF00) >> 8);
        //  byte c2 = (byte)((l & 0x00FF0000) >> 16);
        //  byte c3 = (byte)((l & 0xFF000000) >> 24);
        
        byte c0 = cp[0];
        byte c1 = cp[1];
        byte c2 = cp[2];
        byte c3 = cp[3];
        
        sendByte(c0);
        sendByte(c1);
        sendByte(c2);
        sendByte(c3);
        Serial.flush();
}

void SerialAPI::sendFloat32(float f) {

        byte *cp = (byte *) &f;
        
        byte c0 = cp[0];
        byte c1 = cp[1];
        byte c2 = cp[2];
        byte c3 = cp[3];
        
        sendByte(c0);
        sendByte(c1);
        sendByte(c2);
        sendByte(c3);
        Serial.flush();        
}

void SerialAPI::sendHeader(byte len, byte opcode) {

        sendByte(SERIAL_MAGIC_1);
        sendByte(SERIAL_MAGIC_2);
        sendByte(len);
        sendByte(opcode);
        Serial.flush();
}

void SerialAPI::sendMsgBody(uint8_t *msg, unsigned int len) {

        for (unsigned int i = 0; i < len; i++) {

                sendByte(msg[i]);
        }
}


// CAUTION: These are all blocking receiving methods
uint8_t SerialAPI::recvByte() {
        
        while (Serial.available() <= 0)
                ;
        return (char) Serial.read();
}

uint16_t SerialAPI::recvUInt16() {
        
        char bytearray[2];
        
        bytearray[0] = recvByte();
        bytearray[1] = recvByte();
        
        return *((int *) bytearray);
}

uint32_t SerialAPI::recvUInt32() {

        char bytearray[4];
        
        bytearray[0] = recvByte();
        bytearray[1] = recvByte();
        bytearray[2] = recvByte();
        bytearray[3] = recvByte();
        
        return *((long *) bytearray);
}

float SerialAPI::recvFloat32() {
        
        char bytearray[4];
        
        bytearray[0] = recvByte();
        bytearray[1] = recvByte();
        bytearray[2] = recvByte();
        bytearray[3] = recvByte();
        
        return *((float *) bytearray);
}


void SerialAPI::dispatchCommand(uint8_t opcode, uint8_t len, uint8_t *buf) {

        handlers[opcode](len, buf);
        
}





