#include "serial.hpp"
#include "receiver.hpp"
#include "vesc.hpp"
#include "receiver_data.hpp"

#include "swiftrobotc/swiftrobotc.h"
#include "swiftrobotc/msgs.h"

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

#ifdef __linux__
#ifdef __arm__
// on raspberry
#define SERIAL_RECEIVER "/dev/ttySC0"
#define SERIAL_VESC "/dev/ttySC1"
#else
// on other linux machine with serial adapter
#define SERIAL_RECEIVER "/dev/ttyUSB0"
#define SERIAL_VESC "/dev/ttyUSB1"
#endif
#endif

// swiftrobotm channels
#define SR_INTERNAL (uint16_t)0
#define SR_DRIVE (uint16_t)1
#define SR_STATUS (uint16_t)2

#define TIMEOUT_SWIFTROBOTM 30ms
#define TIMEOUT_RECEIVER 30ms

#define STEERING_MAX_DELTA 0.3
#define STEERING_OFFSET 0.1
#define THROTTLE_MAX_DUTY_CYCLE 0.2
#define FAILSAFE_STEERING 0.6
#define FAILSAFE_DUTYCYCLE 0.0

// global properties
std::unique_ptr<SwiftRobotClient> swiftrobotclient;
std::unique_ptr<Vesc> vesc;
std::unique_ptr<Receiver> receiver;
ReceiverData lastReceiverData;

control_msg::Drive lastControlMsg;
auto lastControlMsgTime = std::chrono::high_resolution_clock::now();
internal_msg::status_t swiftrobotStatus;

std::mutex m;
std::condition_variable cv;

// helper

float convertSteeringRange(float receiverValue) {
    return receiverValue * -STEERING_MAX_DELTA + 0.5 + STEERING_OFFSET;
}

float convertThrottleRange(float receiverValue) {
    return (receiverValue * 0.5 + 0.5) * THROTTLE_MAX_DUTY_CYCLE;
}

void failsafe(std::unique_ptr<Vesc> &vesc) {
    vesc->setServoPos(FAILSAFE_STEERING);
    vesc->setDutyCycle(FAILSAFE_DUTYCYCLE);
}

// callbacks

void receivedVescStatus(VescData data) {
    state_msg::VescStatus msg;
    msg.mosfet_temp = data.mosfet_temp;
    msg.motor_temp = data.motor_temp;
    msg.rpm = data.rpm;
    msg.battery_voltage = data.voltage;
    msg.tachometer = data.ticks;
    msg.tachometer_abs = data.ticksAbs;
    swiftrobotclient->publish(SR_STATUS, msg);

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

void swiftrobotmReceivedInternal(internal_msg::UpdateMsg msg) {
    DBG_PRINT("Device %d is now %d \n", msg.deviceID, msg.status);
    swiftrobotStatus = msg.status;
}

void swiftrobotmReceivedDrive(control_msg::Drive msg) {
    lastControlMsg = msg;
    lastControlMsgTime = std::chrono::high_resolution_clock::now();
}

int main(int argc, char** argv) {
    // factory
    receiver = std::make_unique<Receiver>(SERIAL_RECEIVER, 115200);
    vesc = std::make_unique<Vesc>(SERIAL_VESC, 115200);
    swiftrobotclient = std::make_unique<SwiftRobotClient>(2345); // usb connection

    receiver->setPacketReceivedCallback(&receivedSumDPacket);
    receiver->start();

    vesc->setStatusReceivedCallback(&receivedVescStatus);
    vesc->start();

    swiftrobotclient->subscribe<internal_msg::UpdateMsg>(SR_INTERNAL, &swiftrobotmReceivedInternal);
    swiftrobotclient->subscribe<control_msg::Drive>(SR_DRIVE, &swiftrobotmReceivedDrive);
    swiftrobotclient->start();

    // MAIN LOOP
    while (1) {
        // ****************
        // get data
        // ****************
        // ask for vesc status; response comes async
        vesc->requestState();
        // wait for receiver
        std::unique_lock<std::mutex> l(m);
        if(cv.wait_for(l, TIMEOUT_RECEIVER) == std::cv_status::timeout) {
            //timeout
            failsafe(vesc);
            continue;
        }
        // ****************
        // send data
        // ****************
        // FROM HERE ONLY EXECUTED IF REMOTE IS ON
        if (lastReceiverData.autonomous) {
            // set commands from iOS device
            if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastControlMsgTime) > TIMEOUT_SWIFTROBOTM || swiftrobotStatus != internal_msg::status_t::CONNECTED) {
                failsafe(vesc);
                continue;
            }
            vesc->setServoPos(lastControlMsg.steer);
        } else {
            // remote uses direct control
            vesc->setServoPos(lastReceiverData.steering);
            float throttle = (lastReceiverData.gearSelector != reverse) ? lastReceiverData.throttle : -lastReceiverData.throttle;
            vesc->setDutyCycle(throttle);
        }
    }
}
