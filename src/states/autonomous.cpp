#include "states/autonomous.hpp"
#include "states/base_state.hpp"
#include "context.hpp"

void Autonomous::entry() {
   context_->ledcontroller->turnOnAutonomous();
}

void Autonomous::exit() {
    
}

void Autonomous::manualControl() {
    context_->transitionTo(new Manual_Waiting);
}

void Autonomous::autonomousControl() {
    
}

void Autonomous::lateralControl() {
    if (context_->swiftrobotConnected) {
        context_->transitionTo(new Lateral_Control);
    }
}

void Autonomous::receiverTimedOut() {
    context_->transitionTo(new Fail_Safe);
}

void Autonomous::receiverMotorReset() {
    
}

void Autonomous::swiftrobotTimedOut() {
    context_->transitionTo(new Manual_Waiting);
}

void Autonomous::receiverConnected() {
    
}