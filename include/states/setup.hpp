#ifndef SETUP_HPP
#define SETUP_HPP

#include "base_state.hpp"

class Setup: public BaseState {
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

#endif