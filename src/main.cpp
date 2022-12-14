#include "serial.hpp"
#include "receiver.hpp"
#include "vesc.hpp"
#include "receiver_data.hpp"
#include "timer.hpp"

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
#define SERIAL_RECEIVER "/dev/ttyUSB1"
#define SERIAL_VESC "/dev/ttyUSB0"
#endif
#endif

// swiftrobotm channels
#define SR_INTERNAL (uint16_t) 0x0
#define SR_DRIVE (uint16_t) 0x01
#define SR_STATUS (uint16_t) 0x11
#define SR_RECEIVER (uint16_t) 0x13

#define TIMEOUT_SWIFTROBOTM 30ms
#define TIMEOUT_RECEIVER 30ms

#define INTERVAL_VESCSTATUS_PUBLISH 100 // ms
#define INTERVAL_RECEIVER_PUBLISH 100 // ms

#define STEERING_MAX_DELTA 0.3
#define STEERING_OFFSET 0.1
#define THROTTLE_MAX_DUTY_CYCLE 0.2
#define FAILSAFE_STEERING 0.6
#define FAILSAFE_DUTYCYCLE 0.0

// global properties
std::unique_ptr<SwiftRobotClient> swiftrobotclient;
std::unique_ptr<Vesc> vesc;
std::unique_ptr<Receiver> receiver;
/// timer in which interval the vesc status is published via swiftrobotm
std::unique_ptr<Timer> vescStatusPublishTimer; 
std::unique_ptr<Timer> receiverPublishTimer;

SumD_Packet lastReceiverPacket; // last raw receiver packet
ReceiverData lastReceiverData; // vesc friendly interpretation of raw receiver packet

control_msg::Drive lastControlMsg;
auto lastControlMsgTime = std::chrono::high_resolution_clock::now();
bool swiftrobotConneted;

std::mutex m;
std::condition_variable cv;

// helper

float convertSteeringRange(float receiverValue) {
    return receiverValue * -STEERING_MAX_DELTA + 0.5 + STEERING_OFFSET;
}

float convertThrottleRange(float receiverValue) {
    return (receiverValue * 0.5 + 0.5) * THROTTLE_MAX_DUTY_CYCLE;
}

bool timedOut(std::chrono::_V2::system_clock::time_point toCheck, std::chrono::milliseconds timeout) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - toCheck) > timeout;
}

void failsafe(std::unique_ptr<Vesc> &vesc) {
    vesc->setServoPos(FAILSAFE_STEERING);
    vesc->setDutyCycle(FAILSAFE_DUTYCYCLE);
}

// *************************
// callbacks
// *************************

// callbacks from hardware
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
    lastReceiverPacket = packet;

    lastReceiverData.throttle = convertThrottleRange(Receiver::getPercent(packet, THROTTLE_CHANNEL));
    lastReceiverData.steering = convertSteeringRange(Receiver::getPercent(packet, STEERING_CHANNEL));
    lastReceiverData.gearSelector = (Receiver::getPercent(packet, GEAR_CHANNEL) > 0.0) ? drive : reverse;
    lastReceiverData.autonomous = (Receiver::getPercent(packet, AUTONOMOUS_CHANNEL) > -0.5) ? true : false;
    lastReceiverData.autonomous_deadman = (Receiver::getPercent(packet, AUTONOMOUS_CHANNEL) > 0.5) ? true : false;

    DBG_PRINT("throttle: %f steering: %f gear: %d autonomous: %d, autonomous_deadman: %d \n", lastReceiverData.throttle, lastReceiverData.steering, lastReceiverData.gearSelector, lastReceiverData.autonomous, lastReceiverData.autonomous_deadman);

    cv.notify_one();
}

// swiftrobotm callbacks 
void swiftrobotmReceivedInternal(internal_msg::UpdateMsg msg) {
    DBG_PRINT("Device %d is now %d \n", msg.deviceID, msg.status);
    swiftrobotConneted = (msg.status == internal_msg::status_t::CONNECTED);
}

void swiftrobotmReceivedDrive(control_msg::Drive msg) {
    lastControlMsg = msg;
    lastControlMsgTime = std::chrono::high_resolution_clock::now();
}

// timer callbacks
void timerTriggeredVescStatusPublish() {
    // ask for vesc status; response comes async over callback
    vesc->requestState();
}

void timerTriggeredReceiverPublish() {
    // we send last raw receiver data (not interpreted with receiver_data.hpp)
    base_msg::UInt16Array msg;
    msg.data = std::vector<uint16_t>(lastReceiverPacket.channel, lastReceiverPacket.channel+lastReceiverPacket.numChannels);
    swiftrobotclient->publish(SR_RECEIVER, msg);
}

int main(int argc, char** argv) {
    // factory
    receiver = std::make_unique<Receiver>(SERIAL_RECEIVER, 115200);
    vesc = std::make_unique<Vesc>(SERIAL_VESC, 115200);
    swiftrobotclient = std::make_unique<SwiftRobotClient>(2345); // usb connection
    vescStatusPublishTimer = std::make_unique<Timer>();
    receiverPublishTimer = std::make_unique<Timer>();

    receiver->setPacketReceivedCallback(&receivedSumDPacket);
    receiver->start();

    vesc->setStatusReceivedCallback(&receivedVescStatus);
    vesc->start();

    swiftrobotclient->subscribe<internal_msg::UpdateMsg>(SR_INTERNAL, &swiftrobotmReceivedInternal);
    swiftrobotclient->subscribe<control_msg::Drive>(SR_DRIVE, &swiftrobotmReceivedDrive);
    swiftrobotclient->start();

    vescStatusPublishTimer->setInterval(&timerTriggeredVescStatusPublish, INTERVAL_VESCSTATUS_PUBLISH);
    receiverPublishTimer->setInterval(&timerTriggeredReceiverPublish, INTERVAL_RECEIVER_PUBLISH);

    // MAIN LOOP
    while (1) {
        // wait for receiver
        std::unique_lock<std::mutex> l(m);
        if(cv.wait_for(l, TIMEOUT_RECEIVER) == std::cv_status::timeout) {
            //timeout
   //         failsafe(vesc);
            continue;
        }

        // FROM HERE ONLY EXECUTED IF REMOTE IS ON
        if (lastReceiverData.autonomous) {
            // set commands from iOS device
            if (timedOut(lastControlMsgTime, TIMEOUT_SWIFTROBOTM) || !swiftrobotConneted) {
//                failsafe(vesc);
                continue;
            }
//            vesc->setServoPos(lastControlMsg.steer);
        } else {
            // remote uses direct control
//            vesc->setServoPos(lastReceiverData.steering);
            float throttle = (lastReceiverData.gearSelector != reverse) ? lastReceiverData.throttle : -lastReceiverData.throttle;
//            vesc->setDutyCycle(throttle);
        }
    }
}
