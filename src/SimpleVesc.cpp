#include "SimpleVesc.hpp"
#include "crc.h"

// Communication commands
typedef enum {
    COMM_FW_VERSION = 0,
    COMM_JUMP_TO_BOOTLOADER,
    COMM_ERASE_NEW_APP,
    COMM_WRITE_NEW_APP_DATA,
    COMM_GET_VALUES,
    COMM_SET_DUTY,
    COMM_SET_CURRENT,
    COMM_SET_CURRENT_BRAKE,
    COMM_SET_RPM,
    COMM_SET_POS,
    COMM_SET_HANDBRAKE,
    COMM_SET_DETECT,
    COMM_SET_SERVO_POS,
    COMM_SET_MCCONF,
    COMM_GET_MCCONF,
    COMM_GET_MCCONF_DEFAULT,
    COMM_SET_APPCONF,
    COMM_GET_APPCONF,
    COMM_GET_APPCONF_DEFAULT,
    COMM_SAMPLE_PRINT,
    COMM_TERMINAL_CMD,
    COMM_PRINT,
    COMM_ROTOR_POSITION,
    COMM_EXPERIMENT_SAMPLE,
    COMM_DETECT_MOTOR_PARAM,
    COMM_DETECT_MOTOR_R_L,
    COMM_DETECT_MOTOR_FLUX_LINKAGE,
    COMM_DETECT_ENCODER,
    COMM_DETECT_HALL_FOC,
    COMM_REBOOT,
    COMM_ALIVE,
    COMM_GET_DECODED_PPM,
    COMM_GET_DECODED_ADC,
    COMM_GET_DECODED_CHUK,
    COMM_FORWARD_CAN,
    COMM_SET_CHUCK_DATA,
    COMM_CUSTOM_APP_DATA,
    COMM_NRF_START_PAIRING,
    COMM_GPD_SET_FSW,
    COMM_GPD_BUFFER_NOTIFY,
    COMM_GPD_BUFFER_SIZE_LEFT,
    COMM_GPD_FILL_BUFFER,
    COMM_GPD_OUTPUT_SAMPLE,
    COMM_GPD_SET_MODE,
    COMM_GPD_FILL_BUFFER_INT8,
    COMM_GPD_FILL_BUFFER_INT16,
    COMM_GPD_SET_BUFFER_INT_SCALE,
    COMM_GET_VALUES_SETUP,
    COMM_SET_MCCONF_TEMP,
    COMM_SET_MCCONF_TEMP_SETUP,
    COMM_GET_VALUES_SELECTIVE,
    COMM_GET_VALUES_SETUP_SELECTIVE,
    COMM_EXT_NRF_PRESENT,
    COMM_EXT_NRF_ESB_SET_CH_ADDR,
    COMM_EXT_NRF_ESB_SEND_DATA,
    COMM_EXT_NRF_ESB_RX_DATA,
    COMM_EXT_NRF_SET_ENABLED,
    COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
    COMM_DETECT_APPLY_ALL_FOC,
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
    COMM_ERASE_NEW_APP_ALL_CAN,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN,
    COMM_PING_CAN,
    COMM_APP_DISABLE_OUTPUT,
    COMM_TERMINAL_CMD_SYNC,
    COMM_GET_IMU_DATA,
    COMM_BM_CONNECT,
    COMM_BM_ERASE_FLASH_ALL,
    COMM_BM_WRITE_FLASH,
    COMM_BM_REBOOT,
    COMM_BM_DISCONNECT,
    COMM_BM_MAP_PINS_DEFAULT,
    COMM_BM_MAP_PINS_NRF5X,
    COMM_ERASE_BOOTLOADER,
    COMM_ERASE_BOOTLOADER_ALL_CAN,
    COMM_PLOT_INIT,
    COMM_PLOT_DATA,
    COMM_PLOT_ADD_GRAPH,
    COMM_PLOT_SET_GRAPH,
    COMM_GET_DECODED_BALANCE,
    COMM_BM_MEM_READ,
    COMM_WRITE_NEW_APP_DATA_LZO,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO,
    COMM_BM_WRITE_FLASH_LZO,
    COMM_SET_CURRENT_REL,
    COMM_CAN_FWD_FRAME
} COMM_PACKET_ID;

static void hexdump(uint8_t* buf, int len) {
    for (int i = 0; i < len; ++i) {
        if (buf[i] < 0x10)
            printf(" 0x0");
        else
            printf(" 0x");
        printf("%x", buf[i]);
    }
    printf("\n");
}

float unpack_f16(float scale, uint8_t* buf, int& idx) {
    float tmp = ((buf[idx] << 8) | buf[idx + 1]) / scale;
    idx += 2;
    return tmp;
}
int32_t unpack_i32(uint8_t* buf, int& idx) {
    int32_t tmp = (buf[idx] << 24) | (buf[idx + 1] << 16) |
                  (buf[idx + 2] << 8) | (buf[idx + 3]);
    idx += 4;
    return tmp;
}

Vesc::Vesc(Serial* s): ser(s) {}

void Vesc::setDuty(float duty) {
    uint8_t paket[5];

    int32_t iduty = (int32_t)(duty * 100000);
    paket[0] = COMM_SET_DUTY;
    paket[1] = iduty >> 24;
    paket[2] = iduty >> 16;
    paket[3] = iduty >> 8;
    paket[4] = iduty;

    sendPaket(paket, 5);
}

void Vesc::setServoPos(float pos) {
    // invalid dev address
    uint8_t paket[3];

    int16_t ipos = (int16_t)(pos * 1000);
    paket[0] = COMM_SET_SERVO_POS;
    paket[1] = ipos >> 8;
    paket[2] = ipos;

    sendPaket(paket, 3);
}

void Vesc::requestState() {
    uint8_t paket[5] = {COMM_GET_VALUES_SELECTIVE,
        0x00,
        0x00,
        0x61,
        0x80};
    sendPaket(paket, 5);
}

/// Gets current state from vesc
int Vesc::getState() {
    requestState();
    // wait for reply
    uint8_t buffer[256];
    auto tmp = recvPaket(buffer, 256);
    if (tmp > 0) {
        if (buffer[0] == COMM_GET_VALUES_SELECTIVE) {
            int index = 5;  // skip packetid and mask
            data.rpm = unpack_i32(buffer, index);
            data.voltage = unpack_f16(10, buffer, index);
            data.ticks = unpack_i32(buffer, index);
            data.ticksAbs = unpack_i32(buffer, index);
        }
    }
    return tmp;
}

// LOW LEVEL

int Vesc::recvPaket(uint8_t* buffer, int buflen, int timeout_us) {
    uint32_t startTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    int ind = 0;    // buffer index
    int state = 0;  // paket structure stage
    int len = 0;    // payload length

    // check timeout
    while ((std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - startTime) < timeout_us) {
        // get next byte
        if (ser->available()) {
            uint8_t b = ser->read();
            // Serial.print(state);
            // Serial.print(" b = 0x");
            // Serial.println(b, HEX);
            switch (state) {
                case 0:  // wait startbyte (0x02)
                    if (b == 0x02) {
                        state++;
                    } else {
                        // Serial.print("start error: ");
                        // Serial.println(b, HEX);
                    }
                    break;
                case 1:  // read payload length
                    len = b;
                    state++;
                    break;
                case 2:  // read payload + crc
                    if (ind < len + 1) {
                        buffer[ind++] = b;
                    } else {
                        buffer[ind++] = b;
                        state++;
                    }
                    break;
                case 3:  // read endbyte
                    if (b == 0x03) {
                        // check crc
                        uint16_t calcCRC = crc16(buffer, ind);
                        if (calcCRC == 0) {
                            printf("MSG len: ");
                            printf("%d",len);
                            hexdump(buffer, len);
                            return len;
                        } else {
                            printf("crc error msgCRC: ");
                            printf("%x \n",calcCRC);
                            ind = 0;
                            state = 0;
                        }
                    } else {
                        // Serial.print("endbyte error: ");
                        // Serial.println(b, HEX);
                        ind = 0;
                        state = 0;
                    }
                    break;
                default:  // reset
                    state = 0;
                    ind = 0;
                    break;
            }
        }
    }
    return -1;
}

void Vesc::sendPaket(uint8_t* payload, int len) {
    // long pakets not supported
    if (len > 255) return;
    
    uint16_t crcPayload = crc16(payload, len);
    ser->writeByte(0x02); // start byte
    ser->writeByte(len); // length of payload
    ser->writeBytes(payload, len);
    ser->writeByte((uint8_t)(crcPayload >> 8));
    ser->writeByte((uint8_t)(crcPayload & 0xFF));
    ser->writeByte(3); // end byte
}
