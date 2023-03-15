#ifndef SIMPLE_VESC_HPP
#define SIMPLE_VESC_HPP

#include "serial.hpp"
#include "ringbuffer.hpp"
#include "config.h"

#include <chrono>

#define VESC_PACKET_MAXSIZE 259
#define VESC_PACKET_MINSIZE 5

struct VescData {
    float mosfet_temp = 0;
    float motor_temp = 0;
    int32_t rpm = 0;
    float voltage = 0;
    int32_t ticks = 0;
    int32_t ticksAbs = 0;
};

class Vesc {
public:
    Vesc(std::string dev, uint32_t baud);
    void start();
    void setStatusReceivedCallback(std::function<void(VescData data)> callback);

    /// assert in range [0.0 , 1.0]
    void setDutyCycle(float duty);
    void setCurrent(float current);
    void setCurrentBrake(float current);
    /// assert in range [0.0 , 1.0]
    void setServoPos(float pos);
    void requestState();

    VescData data;

private:
    void sendPaket(uint8_t* payload, int len);
    void uartReceive(uint8_t* buffer, int buflen); // standard timeout is 10 ms
    int analyzePacket();
    uint16_t vesc_crc16(int start, int len);

    float unpack_f16(float scale, int& idx);
    int32_t unpack_i32(int& idx);
private:
    std::unique_ptr<Serial> ser;
    RingBuffer<VESC_PACKET_MAXSIZE> buffer_;
    std::function<void(VescData data)> statusReceivedCallback;
};

#endif  // SIMPLE_VESC_HPP
