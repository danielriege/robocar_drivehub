#pragma once

#include "base_state.hpp"

class Lateral_Control: public BaseState {
    void exit() override;
    void entry() override; 
    
    void manualControl() override;
    void autonomousControl() override;
    void lateralControl() override;
    void receiverTimedOut() override;
    void receiverMotorReset() override;
    void swiftrobotTimedOut() override;
    void receiverConnected() override;
};
