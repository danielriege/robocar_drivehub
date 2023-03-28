#include "config.h"
#include "serial.hpp"
#include "receiver.hpp"
#include "vesc.hpp"
#include "timer.hpp"
#include "ledcontroller.hpp"

#include "swiftrobotc/swiftrobotc.h"
#include "swiftrobotc/msgs.h"

// FSM
#include "states/base_state.hpp"
#include "states/setup.hpp"
#include "context.hpp"

#include <unistd.h>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <mutex>

using namespace std::chrono_literals;

#define DEBUGGING
#ifdef DEBUGGING
#define DBG_PRINT(x...) printf(x)
#else
#define DBG_PRINT(x...) //
#endif

// global properties
std::unique_ptr<Context> context;

std::shared_ptr<SwiftRobotClient> swiftrobotclient;
std::shared_ptr<Vesc> vesc;
std::shared_ptr<Receiver> receiver;
std::shared_ptr<LEDController> ledcontroller;
/// timer in which interval the vesc status is published via swiftrobotm
std::unique_ptr<Timer> vescStatusPublishTimer; 

auto lastSwiftrobotPing = std::chrono::high_resolution_clock::now();
auto lastReceiverPing = std::chrono::high_resolution_clock::now();

std::mutex m_context;

// helper
bool timedOut(std::chrono::_V2::system_clock::time_point toCheck, std::chrono::milliseconds timeout) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - toCheck) > timeout;
}

// *************************
// callbacks
// *************************

// callbacks from hardware
void receivedVescStatus(VescData data) {
    base_msg::UInt32Array msg;
    // cast our packet into a uint16_t vector
    std::vector<uint32_t> ser_vesc;
    // for float we are using the same scales as VESC (e.g. vesc.cpp->analyzePacket())
    ser_vesc.push_back((uint32_t)(data.mosfet_temp*10));
    ser_vesc.push_back((uint32_t)(data.motor_temp*10));
    ser_vesc.push_back((uint32_t) data.rpm);
    ser_vesc.push_back((uint32_t) (data.voltage*10));
    ser_vesc.push_back((uint32_t) data.ticks);
    ser_vesc.push_back((uint32_t) data.ticksAbs);
    msg.data = ser_vesc;
    swiftrobotclient->publish(SR_STATUS, msg);
}

void receivedReceiverPacket(ReceiverPacket packet) {
    lastReceiverPing = std::chrono::high_resolution_clock::now();
    m_context.lock();
    context->updateReceiverPacket(packet);
    context->receiverConnected();
    // send triggers initiated by receiver
    if (packet.lateral_control) {
        if (packet.autonomous) {
            context->autonomousControl();
        } else {
            context->lateralControl();
        }
    } else {
        context->manualControl();
    }
    if (packet.throttle <= 0.005) {
        context->receiverMotorReset();
    }
    m_context.unlock();


    // now forward our packet to the iOS Device
    base_msg::UInt16Array msg;
    // cast our packet into a uint16_t vector
    std::vector<uint16_t> ser_packet;
    ser_packet.push_back((uint16_t)(packet.throttle*65000)); // this maps throttle to 0-65000
    ser_packet.push_back((uint16_t)(packet.steering*65000)); // this maps steering to 0-65000
    ser_packet.push_back((uint16_t) packet.gearSelector); // 0 = undefined, 1 = drive, 2 = reverse
    ser_packet.push_back((uint16_t) packet.lateral_control); // 0 false, 1 true
    ser_packet.push_back((uint16_t) packet.autonomous); // 0 false, 1 true
    msg.data = ser_packet;
    swiftrobotclient->publish(SR_RECEIVER, msg);
}

// swiftrobotm callbacks 
void swiftrobotmReceivedInternal(internal_msg::UpdateMsg msg) {
    DBG_PRINT("Device %d is now %d \n", msg.deviceID, msg.status);
    m_context.lock();
    context->swiftrobotConnected = (msg.status == internal_msg::status_t::CONNECTED);
    m_context.unlock();
}

void swiftrobotmReceivedDrive(control_msg::Drive msg) {
    lastSwiftrobotPing = std::chrono::high_resolution_clock::now();
     m_context.lock();
    context->updateDriveMsg(msg);
     m_context.unlock();
}

// timer callbacks
void timerTriggeredVescStatusPublish() {
    // ask for vesc status; response comes async over callback
    vesc->requestState();
}

int main(int argc, char** argv) {
    // construct objects
    ledcontroller = std::make_shared<LEDController>();
    receiver = std::make_shared<Receiver>(SERIAL_RECEIVER, 115200);
    vesc = std::make_shared<Vesc>(SERIAL_VESC, 115200);
    swiftrobotclient = std::make_shared<SwiftRobotClient>(2345); // usb connection

    vescStatusPublishTimer = std::make_unique<Timer>();

    // start FSM in setup
    context = std::make_unique<Context>(new Setup, swiftrobotclient, vesc, receiver, ledcontroller); // setup is dummy state to signal we are in setup even though everything happens here...

    receiver->setPacketReceivedCallback(&receivedReceiverPacket);
    receiver->start();

    vesc->setStatusReceivedCallback(&receivedVescStatus);
    vesc->start();

    swiftrobotclient->subscribe<internal_msg::UpdateMsg>(SR_INTERNAL, &swiftrobotmReceivedInternal);
    swiftrobotclient->subscribe<control_msg::Drive>(SR_DRIVE, &swiftrobotmReceivedDrive);
    swiftrobotclient->start();

    vescStatusPublishTimer->setInterval(&timerTriggeredVescStatusPublish, INTERVAL_VESCSTATUS_PUBLISH);

    context->swiftrobotConnected = true;

    ledcontroller->turnOnDaylight();

    // watchdog
    while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(INTERVAL_TIMEOUT_CHECK));
        m_context.lock();
        if (timedOut(lastReceiverPing, TIMEOUT_HARDWARE)) {
            context->receiverTimedOut();
        }
        if (timedOut(lastSwiftrobotPing, TIMEOUT_HARDWARE)) {
//            context->swiftrobotTimedOut();
        }
        m_context.unlock();
    }
}
