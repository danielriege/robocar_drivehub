#pragma once

#include <cstdint>
#include "ringbuffer.hpp"
#include "serial.hpp"
#include "crc.h"

#define MAX_CHAN_COUNT (16)
#define SUMD_PAKET_MINSIZE (3 + 24 + 2)
#define MAN_ID (0xA8)
#define STATE_NORMAL (0x01)
#define STATE_FS (0x81)

struct SumD_Packet{
    uint8_t manufactureId;
    uint8_t state;
    uint8_t numChannels;
    uint16_t channel[MAX_CHAN_COUNT] = {12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000};
    uint16_t crc;
};

class Receiver {
public:
    Receiver(std::string dev, uint32_t baud);
    void start();
    void setPacketReceivedCallback(std::function<void(SumD_Packet data)> callback);
    
    static inline float getPercent(SumD_Packet packet, int chan) { return (float)(packet.channel[chan] - 12000) / 3200; }
    static inline uint16_t getRaw(SumD_Packet packet, int chan) { return packet.channel[chan]; }
    static inline uint16_t getPPM(SumD_Packet packet, int chan) { return packet.channel[chan] >> 3; }
private:
    void uartReceive(uint8_t* data, size_t size);
    int analyzePacket();
    uint16_t sumd_crc16(int len);
private:
    RingBuffer<64> buffer_;
    uint32_t recvNumChannels = 0;

    SumD_Packet packet;
    std::unique_ptr<Serial> ser;
    std::function<void(SumD_Packet data)> packetReceivedCallback;
};
