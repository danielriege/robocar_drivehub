#ifndef CONTEXT_HPP
#define CONTEXT_HPP

#include "states/base_state.hpp"
#include "receiver.hpp"
#include "vesc.hpp"
#include "receiver_data.hpp"
#include "timer.hpp"
#include "ledcontroller.hpp"

#include "swiftrobotc/swiftrobotc.h"
#include "swiftrobotc/msgs.h"

#include <unistd.h>

class Context {
private:
    BaseState *state_;
public:
    // application properties
    std::shared_ptr<SwiftRobotClient> swiftrobotclient;
    std::shared_ptr<Vesc> vesc;
    std::shared_ptr<Receiver> receiver;
    std::shared_ptr<LEDController> ledcontroller;

    // flags
    bool swiftrobotConnected;
public: 
    Context(BaseState *state, 
            std::shared_ptr<SwiftRobotClient> &swiftrobotclient,
            std::shared_ptr<Vesc> &vesc,
            std::shared_ptr<Receiver> &receiver,
            std::shared_ptr<LEDController> &ledcontroller): state_(nullptr) {
        this->transitionTo(state);
        this->swiftrobotclient = swiftrobotclient;
        this->vesc = vesc;
        this->receiver = receiver;
        this->ledcontroller = ledcontroller;
        this->swiftrobotConnected = false;
    }

    ~Context() {
        delete state_;
    }

    void transitionTo(BaseState *state) {
        if (this->state_ != nullptr) {
            this->state_->exit();
            delete this->state_;
        }
        this->state_ = state;
        this->state_->set_context(this);
        this->state_->entry();
    }

    void transisitionToHistory() {
        
    }

    void manualControl() {
        this->state_->manualControl();
    }

    void autonomousControl() {
        this->state_->autonomousControl();
    }

    void lateralControl() {
        this->state_->lateralControl();
    }

    void receiverTimedOut() {
        this->state_->receiverTimedOut();
    }

    void receiverMotorReset() {
        this->state_->receiverMotorReset();
    }

    void swiftrobotTimedOut() {
        this->state_->swiftrobotTimedOut();
    }

    void receiverConnected() {
        this->state_->receiverConnected();
    }
};

#endif