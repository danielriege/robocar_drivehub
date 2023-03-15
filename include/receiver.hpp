#pragma once

#include <cstdint>
#include "ringbuffer.hpp"
#include "serial.hpp"
#include "crc.h"
#include "config.h"

#define MAX_CHAN_COUNT (16)
#define SUMD_PAKET_MINSIZE (3 + 24 + 2)
#define MAN_ID (0xA8)
#define STATE_NORMAL (0x01)
#define STATE_FS (0x81)

#define THROTTLE_CHANNEL 1
#define STEERING_CHANNEL 2
#define GEAR_CHANNEL 3
#define AUTONOMOUS_CHANNEL 4

struct SumD_Packet{
    uint8_t manufactureId;
    uint8_t state;
    uint8_t numChannels;
    uint16_t channel[MAX_CHAN_COUNT] = {12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000, 12000};
    uint16_t crc;
};

enum ReceiverGear {
    undefined, drive, reverse
};

struct ReceiverPacket {
    /// converted into car specific range [0.0, THROTTLE_MAX_DUTY_CYCLE]
    float throttle = 0;
    /// converted into car specific range [0.0, 1.0]. center depends on offset
    float steering = 0;
    ReceiverGear gearSelector = undefined;
    bool lateral_control = 0;
    bool autonomous = 0;
};

class Receiver {
public:
    Receiver(std::string dev, uint32_t baud);
    void start();
    void setPacketReceivedCallback(std::function<void(ReceiverPacket packet)> callback);
private:
    static inline float getPercent(SumD_Packet packet, int chan) { return (float)(packet.channel[chan] - 12000) / 3200; }
    static inline uint16_t getRaw(SumD_Packet packet, int chan) { return packet.channel[chan]; }
    static inline uint16_t getPPM(SumD_Packet packet, int chan) { return packet.channel[chan] >> 3; }

    static inline float convertSteeringRange(float receiverValue) { return receiverValue * -STEERING_MAX_DELTA + 0.5 + STEERING_OFFSET; }

    static inline float convertThrottleRange(float receiverValue) { return (receiverValue * 0.5 + 0.5) * THROTTLE_MAX_DUTY_CYCLE; }


    void uartReceive(uint8_t* data, size_t size);
    int analyzePacket();
    uint16_t sumd_crc16(int len);
    void SumD_to_ReceiverPacket(SumD_Packet sumd, ReceiverPacket *packet);
private:
    RingBuffer<64> buffer_;
    uint32_t recvNumChannels = 0;

    SumD_Packet packet;
    std::unique_ptr<Serial> ser;
    std::function<void(ReceiverPacket packet)> packetReceivedCallback;
};
