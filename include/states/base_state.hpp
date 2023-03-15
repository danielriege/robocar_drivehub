#ifndef MAIN_FSM_HPP
#define MAIN_FSM_HPP

#include "swiftrobotc/swiftrobotc.h"
#include "swiftrobotc/msgs.h"
#include "receiver.hpp"

class Context;

class BaseState {
protected:
    Context* context_;
public:
    virtual ~BaseState() {

    }

    void set_context(Context* context) {
        this->context_ = context;
    }

    virtual void exit() = 0;
    virtual void entry() = 0;

     // update events
    virtual void ReceiverPacketUpdated(ReceiverPacket packet) = 0;
    virtual void DriveMsgUpdated(control_msg::Drive msg) = 0;

    // input signals
    virtual void manualControl() = 0;
    virtual void autonomousControl() = 0;
    virtual void lateralControl() = 0;
    virtual void receiverTimedOut() = 0;
    virtual void receiverMotorReset() = 0;
    virtual void swiftrobotTimedOut() = 0;
    virtual void receiverConnected() = 0;
};

#endif