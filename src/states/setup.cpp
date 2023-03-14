#include "states/base_state.hpp"
#include "states/setup.hpp"
#include "states/manual_waiting.hpp"
#include "context.hpp"

void Setup::entry() {

}

void Setup::exit() {
    context_->ledcontroller->signalSetupComplete();
}

void Setup::manualControl() {
    context_->transitionTo(new ManualWaiting);
}

void Setup::autonomousControl() {
    
}

void Setup::lateralControl() {
    
}

void Setup::receiverTimedOut() {
    
}

void Setup::receiverMotorReset() {
    
}

void Setup::swiftrobotTimedOut() {
    
}

void Setup::receiverConnected() {
    
}