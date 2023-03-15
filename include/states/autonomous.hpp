#pragma once

#include "base_state.hpp"

class Autonomous: public BaseState {
    void exit() override;
    void entry() override; 

    void ReceiverPacketUpdated(ReceiverPacket packet) override;
    void DriveMsgUpdated(control_msg::Drive msg) override; 
    
    void manualControl() override;
    void autonomousControl() override;
    void lateralControl() override;
    void receiverTimedOut() override;
    void receiverMotorReset() override;
    void swiftrobotTimedOut() override;
    void receiverConnected() override;
};
