#include "states/autonomous.hpp"
#include "states/base_state.hpp"
#include "states/manual_waiting.hpp"
#include "states/lateral_control.hpp"
#include "states/fail_safe.hpp"
#include "context.hpp"

void Autonomous::entry() {
   context_->ledcontroller->turnOnAutonomous();
}

void Autonomous::exit() {}

void Autonomous::manualControl() {
    context_->transitionTo(new Manual_Waiting);
}

void Autonomous::autonomousControl() {}

void Autonomous::lateralControl() {
    if (context_->swiftrobotConnected) {
        context_->transitionTo(new Lateral_Control);
    }
}

void Autonomous::receiverTimedOut() {
    context_->transitionTo(new Fail_Safe);
}

void Autonomous::receiverMotorReset() {}

void Autonomous::swiftrobotTimedOut() {
    context_->transitionTo(new Manual_Waiting);
}

void Autonomous::receiverConnected() {}

void Autonomous::ReceiverPacketUpdated(ReceiverPacket packet) {}

void Autonomous::DriveMsgUpdated(control_msg::Drive msg) {
    context_->vesc->setServoPos(msg.steer);
    float dutyCycle = (msg.reverse == false) ? msg.throttle : -msg.throttle;
    context_->vesc->setDutyCycle(dutyCycle);
}