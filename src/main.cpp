#include "serial.hpp"
#include "receiver.hpp"
#include "vesc.hpp"
#include "receiver_data.hpp"

#include <unistd.h>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <mutex>

using namespace std::chrono_literals;

//#define DEBUGGING
#ifdef DEBUGGING
#define DBG_PRINT(x...) printf(x)
#else
#define DBG_PRINT(x...) //
#endif

#define TIMEOUT_RECEIVER 30ms
#define STEERING_MAX_DELTA 0.3
#define STEERING_OFFSET 0.1
#define THROTTLE_MAX_DUTY_CYCLE 0.1

// global properties
VescData lastVescData;
ReceiverData lastReceiverData;

std::mutex m;
std::condition_variable cv;

// helper

float convertSteeringRange(float receiverValue) {
    return receiverValue * -STEERING_MAX_DELTA + 0.5 + STEERING_OFFSET;
}

float convertThrottleRange(float receiverValue) {
    return (receiverValue * 0.5 + 0.5) * THROTTLE_MAX_DUTY_CYCLE;
}

// callbacks

void receivedVescStatus(VescData data) {
    lastVescData = data;
    DBG_PRINT("mosfet temp: %f, motor temp: %f, rpm: %d, voltage: %f, tachometer: %d, tachometer abs: %d \n",
        data.mosfet_temp, data.motor_temp, data.rpm, data.voltage, data.ticks, data.ticksAbs);
}

void receivedSumDPacket(SumD_Packet packet) {
    lastReceiverData.throttle = convertThrottleRange(Receiver::getPercent(packet, THROTTLE_CHANNEL));
    lastReceiverData.steering = convertSteeringRange(Receiver::getPercent(packet, STEERING_CHANNEL));
    lastReceiverData.gearSelector = (Receiver::getPercent(packet, GEAR_CHANNEL) > 0.0) ? drive : reverse;
    lastReceiverData.autonomous = (Receiver::getPercent(packet, AUTONOMOUS_CHANNEL) > -0.5) ? true : false;
    lastReceiverData.autonomous_deadman = (Receiver::getPercent(packet, AUTONOMOUS_CHANNEL) > 0.5) ? true : false;

    DBG_PRINT("throttle: %f steering: %f gear: %d autonomous: %d, autonomous_deadman: %d \n", lastReceiverData.throttle, lastReceiverData.steering, lastReceiverData.gearSelector, lastReceiverData.autonomous, lastReceiverData.autonomous_deadman);

    cv.notify_one();
}

int main(int argc, char** argv) {
    // factory
    Receiver receiver("/dev/ttyUSB1", 115200);
    Vesc vesc("/dev/ttyUSB0", 115200);

    receiver.setPacketReceivedCallback(&receivedSumDPacket);
    receiver.start();

    vesc.setStatusReceivedCallback(&receivedVescStatus);
    vesc.start();

    // MAIN LOOP
    while (1) {
        // wait for receiver
        std::unique_lock<std::mutex> l(m);
        if(cv.wait_for(l, TIMEOUT_RECEIVER) == std::cv_status::timeout) {
            //timeout
            vesc.setServoPos(0.5);
            vesc.setDutyCycle(0.0);
            continue;
        }
        // web have new receiver data
        if (lastReceiverData.autonomous) {
            // set commands from iOS device
        } else {
            vesc.setServoPos(lastReceiverData.steering);
            float throttle = (lastReceiverData.gearSelector != reverse) ? lastReceiverData.throttle : -lastReceiverData.throttle;
            vesc.setDutyCycle(throttle);
        }
    }
}
