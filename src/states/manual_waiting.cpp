#include "states/manual_waiting.hpp"
#include "states/base_state.hpp"
#include "states/autonomous.hpp"
#include "states/lateral_control.hpp"
#include "states/fail_safe.hpp"
#include "states/manual_control.hpp"
#include "context.hpp"

void Manual_Waiting::entry() {
    context_->ledcontroller->turnOffAutonomous();
    context_->vesc->setServoPos(FAILSAFE_STEERING);
    context_->vesc->setDutyCycle(FAILSAFE_DUTYCYCLE);
    printf("entry manual waiting\n");
}

void Manual_Waiting::exit() {
    printf("exit manual waiting\n");
}

void Manual_Waiting::manualControl() {}

void Manual_Waiting::autonomousControl() {
    if (context_->swiftrobotConnected) {
        context_->transitionTo(new Autonomous);
    }
}

void Manual_Waiting::lateralControl() {
    if (context_->swiftrobotConnected) {
        context_->transitionTo(new Lateral_Control);
    }
}

void Manual_Waiting::receiverTimedOut() {
    context_->transitionTo(new Fail_Safe);
}

void Manual_Waiting::receiverMotorReset() {
    context_->transitionTo(new Manual_Control);
}

void Manual_Waiting::swiftrobotTimedOut() {}

void Manual_Waiting::receiverConnected() {}

void Manual_Waiting::ReceiverPacketUpdated(ReceiverPacket packet) {}

void Manual_Waiting::DriveMsgUpdated(control_msg::Drive msg) {}