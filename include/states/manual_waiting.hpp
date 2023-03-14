#ifndef MANUAL_WAITING_HPP
#define MANUAL_WAITING_HPP

#include "base_state.hpp"

class ManualWaiting: public BaseState {
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