#include "states/base_state.hpp"
#include "states/setup.hpp"
#include "states/manual_waiting.hpp"
#include "states/autonomous.hpp"
#include "states/lateral_control.hpp"
#include "states/fail_safe.hpp"
#include "context.hpp"

void Setup::entry() {}

void Setup::exit() {
    context_->ledcontroller->signalSetupComplete();
    printf("exit setup\n");
}

void Setup::manualControl() {
    context_->transitionTo(new Manual_Waiting);
}

void Setup::autonomousControl() {
    if (context_->swiftrobotConnected) {
        context_->transitionTo(new Autonomous);
    }
}

void Setup::lateralControl() {
    if (context_->swiftrobotConnected) {
        context_->transitionTo(new Lateral_Control);
    }
}

void Setup::receiverTimedOut() {
    context_->transitionTo(new Fail_Safe);
}

void Setup::receiverMotorReset() {}

void Setup::swiftrobotTimedOut() {}

void Setup::receiverConnected() {}


void Setup::ReceiverPacketUpdated(ReceiverPacket packet) {}

void Setup::DriveMsgUpdated(control_msg::Drive msg) {}