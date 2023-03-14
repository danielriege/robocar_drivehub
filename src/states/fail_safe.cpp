#include "states/fail_safe.hpp"
#include "states/base_state.hpp"
#include "context.hpp"

void Fail_Safe::entry() {
   //context_->ledcontroller->turnOnHazard();
}

void Fail_Safe::exit() {
    //context_->ledcontroller->turnOffHazard();
}

void Fail_Safe::manualControl() {
    
}

void Fail_Safe::autonomousControl() {
    
}

void Fail_Safe::lateralControl() {
    
}

void Fail_Safe::receiverTimedOut() {
    
}

void Fail_Safe::receiverMotorReset() {
    
}

void Fail_Safe::swiftrobotTimedOut() {
    
}

void Fail_Safe::receiverConnected() {
    
}