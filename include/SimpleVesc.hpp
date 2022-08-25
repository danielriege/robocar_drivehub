#ifndef SIMPLE_VESC_HPP
#define SIMPLE_VESC_HPP

#include "serial.hpp"

#include <chrono>

struct VescData {
    float voltage = 0;
    float rpm = 0;
    int32_t ticks = 0;
    int32_t ticksAbs = 0;
};

class Vesc {
    Serial* ser;
    void sendPaket(uint8_t* payload, int len);
    int recvPaket(uint8_t* buffer, int buflen, int timeout_us = 10000); // standard timeout is 10 ms
    void requestState();

   public:
    VescData data;
    Vesc(Serial* s);
    void setDuty(float duty);
    void setServoPos(float pos);
    int getState();
};

#endif  // SIMPLE_VESC_HPP
