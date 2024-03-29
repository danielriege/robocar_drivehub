#include "states/lateral_control.hpp"
#include "states/base_state.hpp"
#include "states/manual_waiting.hpp"
#include "states/fail_safe.hpp"
#include "states/autonomous.hpp"
#include "context.hpp"

void Lateral_Control::entry() {
   context_->ledcontroller->turnOnLateral();
   printf("entry lateral\n");
}

void Lateral_Control::exit() {}

void Lateral_Control::manualControl() {
    context_->transitionTo(new Manual_Waiting);
}

void Lateral_Control::autonomousControl() {
    if (context_->swiftrobotConnected) {
        context_->transitionTo(new Autonomous);
    }
}

void Lateral_Control::lateralControl() {}

void Lateral_Control::receiverTimedOut() {
    context_->transitionTo(new Fail_Safe);
}

void Lateral_Control::receiverMotorReset() {}

void Lateral_Control::swiftrobotTimedOut() {}

void Lateral_Control::receiverConnected() {}

void Lateral_Control::ReceiverPacketUpdated(ReceiverPacket packet) {
    float dutyCycle = (packet.gearSelector != reverse) ? packet.throttle : -packet.throttle;
    context_->vesc->setDutyCycle(dutyCycle);
}

void Lateral_Control::DriveMsgUpdated(control_msg::Drive msg) {
    context_->vesc->setServoPos(msg.steer);
}
