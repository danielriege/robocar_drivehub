#include "states/manual_control.hpp"
#include "states/base_state.hpp"
#include "states/autonomous.hpp"
#include "states/lateral_control.hpp"
#include "states/fail_safe.hpp"
#include "context.hpp"

void Manual_Control::entry() {}

void Manual_Control::exit() {}

void Manual_Control::manualControl() {}

void Manual_Control::autonomousControl() {
    if (context_->swiftrobotConnected) {
        context_->transitionTo(new Autonomous);
    }
}

void Manual_Control::lateralControl() {
    if (context_->swiftrobotConnected) {
        context_->transitionTo(new Lateral_Control);
    }
}

void Manual_Control::receiverTimedOut() {
    context_->transitionTo(new Fail_Safe);
}

void Manual_Control::receiverMotorReset() {}

void Manual_Control::swiftrobotTimedOut() {}

void Manual_Control::receiverConnected() {}

void Manual_Control::ReceiverPacketUpdated(ReceiverPacket packet) {
    context_->vesc->setServoPos(packet.steering);
    float dutyCycle = (packet.gearSelector != reverse) ? packet.throttle : -packet.throttle;
    context_->vesc->setDutyCycle(dutyCycle);
}

void Manual_Control::DriveMsgUpdated(control_msg::Drive msg) {}
