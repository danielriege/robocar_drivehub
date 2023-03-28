#include "states/fail_safe.hpp"
#include "states/base_state.hpp"
#include "context.hpp"

void Fail_Safe::entry() {
   context_->ledcontroller->turnOnHazardLights();
   context_->vesc->setServoPos(FAILSAFE_STEERING);
   context_->vesc->setDutyCycle(FAILSAFE_DUTYCYCLE);
   printf("entry failsafe\n");
}

void Fail_Safe::exit() {
    context_->ledcontroller->turnOffHazardLights();
    printf("exit failsafe\n");
}

void Fail_Safe::manualControl() {}

void Fail_Safe::autonomousControl() {}

void Fail_Safe::lateralControl() {}

void Fail_Safe::receiverTimedOut() {}

void Fail_Safe::receiverMotorReset() {}

void Fail_Safe::swiftrobotTimedOut() {}

void Fail_Safe::receiverConnected() {
    context_->transitionToHistory();
}

void Fail_Safe::ReceiverPacketUpdated(ReceiverPacket packet) {}

void Fail_Safe::DriveMsgUpdated(control_msg::Drive msg) {}
