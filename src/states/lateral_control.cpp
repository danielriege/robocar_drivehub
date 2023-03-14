#include "states/lateral_control.hpp"
#include "states/base_state.hpp"
#include "context.hpp"

void Lateral_Control::entry() {
   context_->ledcontroller->turnOnLateral();
}

void Lateral_Control::exit() {
    
}

void Lateral_Control::manualControl() {
    context_->transitionTo(new Manual_Waiting);
}

void Lateral_Control::autonomousControl() {
    if (context_->swiftrobotConnected) {
        context_->transitionTo(new Autonomous);
    }
}

void Lateral_Control::lateralControl() {

}

void Lateral_Control::receiverTimedOut() {
    context_->transitionTo(new Fail_Safe);
}

void Lateral_Control::receiverMotorReset() {
    
}

void Lateral_Control::swiftrobotTimedOut() {
    
}

void Lateral_Control::receiverConnected() {
    
}