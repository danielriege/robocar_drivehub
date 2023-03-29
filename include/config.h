#pragma once

#include <chrono>

using namespace std::chrono_literals;

#ifdef __linux__
#ifdef __arm__
// on raspberry
#define SERIAL_RECEIVER "/dev/ttySC0"
#define SERIAL_VESC "/dev/ttySC1"
#else
// on other linux machine with serial adapter
#define SERIAL_RECEIVER "/dev/ttyUSB1"
#define SERIAL_VESC "/dev/ttyUSB0"
#endif
#endif

// swiftrobotm channels
#define SR_INTERNAL (uint16_t) 0x0
#define SR_DRIVE (uint16_t) 0x01
#define SR_STATUS (uint16_t) 0x11
#define SR_RECEIVER (uint16_t) 0x13

#define TIMEOUT_HARDWARE 50ms

#define INTERVAL_TIMEOUT_CHECK 50 // ms
#define INTERVAL_VESCSTATUS_PUBLISH 100 // ms

#define STEERING_MAX_DELTA 0.3
#define STEERING_OFFSET 0.1
#define THROTTLE_MAX_DUTY_CYCLE 0.2
#define FAILSAFE_STEERING 0.6
#define FAILSAFE_DUTYCYCLE 0.0