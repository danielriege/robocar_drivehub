#include "vesc.hpp"
#include "crc.h"

//#define DEBUGGING
#ifdef DEBUGGING
#define DBG_PRINT(x...) printf(x)
#else
#define DBG_PRINT(x...) //
#endif

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

// sets callback so program can be notified on new packet
void Vesc::setStatusReceivedCallback(std::function<void(VescData data)> callback) {
    statusReceivedCallback = callback;
}


float Vesc::unpack_f16(float scale, int& idx) {
    float tmp = ((buffer_[idx] << 8) | buffer_[idx + 1]) / scale;
    idx += 2;
    return tmp;
}
int32_t Vesc::unpack_i32(int& idx) {
    int32_t tmp = (buffer_[idx] << 24) | (buffer_[idx + 1] << 16) |
                  (buffer_[idx + 2] << 8) | (buffer_[idx + 3]);
    idx += 4;
    return tmp;
}

Vesc::Vesc(std::string dev, uint32_t baud): ser(std::make_unique<Serial>(dev, baud)) {}

void Vesc::start() {
    ser->startAsync(std::bind(&Vesc::uartReceive, this, std::placeholders::_1, std::placeholders::_2));
}

void Vesc::setDutyCycle(float duty) {
    if (duty >= 0.0 && duty <= 1.0) {
        // convert into car specific bounds
        duty = duty * THROTTLE_MAX_DUTY_CYCLE;
        uint8_t paket[5];

        int32_t iduty = (int32_t)(duty * 100000);
        paket[0] = COMM_SET_DUTY;
        paket[1] = iduty >> 24;
        paket[2] = iduty >> 16;
        paket[3] = iduty >> 8;
        paket[4] = iduty;

        sendPaket(paket, 5);
    }
}

void Vesc::setCurrent(float current) {
    uint8_t paket[5];

    int32_t iduty = (int32_t)(current * 1000);
    paket[0] = COMM_SET_CURRENT;
    paket[1] = iduty >> 24;
    paket[2] = iduty >> 16;
    paket[3] = iduty >> 8;
    paket[4] = iduty;

    sendPaket(paket, 5);
}

void Vesc::setCurrentBrake(float current) {
    uint8_t paket[5];

    int32_t iduty = (int32_t)(current * 1000);
    paket[0] = COMM_SET_CURRENT_BRAKE;
    paket[1] = iduty >> 24;
    paket[2] = iduty >> 16;
    paket[3] = iduty >> 8;
    paket[4] = iduty;

    sendPaket(paket, 5);
}

void Vesc::setServoPos(float pos) {
    if (pos >= 0.0 && pos <= 1.0) {
        // convert into car specific bounds
        pos = (pos * 2 - 1) * -STEERING_MAX_DELTA + 0.5 + STEERING_OFFSET;

        uint8_t paket[3];

        int16_t ipos = (int16_t)(pos * 1000);
        paket[0] = COMM_SET_SERVO_POS;
        paket[1] = ipos >> 8;
        paket[2] = ipos;

        sendPaket(paket, 3);
    }
}

void Vesc::requestState() {
    uint8_t paket[5] = {COMM_GET_VALUES_SELECTIVE,
        0x00,
        0x00,
        0x61,
        0x83};
    sendPaket(paket, 5);
}

// LOW LEVEL

uint16_t Vesc::vesc_crc16(int start, int len) {
    unsigned int i;
	unsigned short cksum = 0;
	for (i = start; i < len+start; i++) {
		cksum = crc16_tab[(((cksum >> 8) ^ buffer_[i]) & 0xFF)] ^ (cksum << 8);
	}
	return cksum;
}

int Vesc::analyzePacket() {
    int len = 0;
    while(buffer_.available() >= VESC_PACKET_MINSIZE + len) {
        // check for start byte
        if (buffer_[0] != 0x02) {
            buffer_.pop(0); // throw away until start byte is right
            continue;
        }
        // get payload length
        len = buffer_[1];
        // check if enough bytes
        if (buffer_.available() < VESC_PACKET_MINSIZE + len) {
            continue;
        }
        // check packet id
        int id = buffer_[2];
        switch (id)
        {
        case COMM_GET_VALUES_SELECTIVE: {
            // copy data
            int index = 7; // start + length + id + 4 bytes mask
            data.mosfet_temp = unpack_f16(10, index);
            data.motor_temp = unpack_f16(10, index);
            data.rpm = unpack_i32(index);
            data.voltage = unpack_f16(10,index);
            data.ticks = unpack_i32(index);
            data.ticksAbs = unpack_i32(index);
            break;
        }
        default:
            break;
        }
        // check crc
        uint16_t calcCRC = vesc_crc16(2, len+2); // from payload until crc (included)
        if (calcCRC != 0) {
            DBG_PRINT("VESC: CRC failed! %d \n", calcCRC);
            hexdump(buffer_.buffer.data(), len);
            buffer_.pop(len+4);
            continue;
        } 
        // check end byte
        if (buffer_[len+4] == 0x03) {
            buffer_.pop(len+5);
            return len;
        }
    }
    return -1;
}

void Vesc::uartReceive(uint8_t* data, int size) {
    for (int i = 0; i < size; i++) {
        buffer_.push(data[i]);
    }
    DBG_PRINT("new uart packet \n");
    if (analyzePacket() >= 0) {
        DBG_PRINT("status packet \n");
        statusReceivedCallback(this->data);
    }
}

void Vesc::sendPaket(uint8_t* payload, int len) {
    // long pakets not supported
    if (len > 255) return;
    
    uint16_t crcPayload = crc16(payload, len);
    uint8_t packet[len+5];
    packet[0] = 0x02;
    packet[1] = (uint8_t)len;
    memcpy(packet+2, payload, len);
    packet[len+2] = (uint8_t)(crcPayload >> 8);
    packet[len+3] = (uint8_t)(crcPayload & 0xFF);
    packet[len+4] = 0x03;

    ser->writeBytes(packet, len+5);
}
