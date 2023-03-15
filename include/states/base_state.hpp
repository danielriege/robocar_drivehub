#ifndef MAIN_FSM_HPP
#define MAIN_FSM_HPP

class Context;

class BaseState {
protected:
    Context* context_;
public:
    virtual ~BaseState() {

    }

    void set_context(Context* context) {
        this->context_ = context;
    }

    virtual void exit() = 0;
    virtual void entry() = 0;

    // Input
    virtual void manualControl() = 0;
    virtual void autonomousControl() = 0;
    virtual void lateralControl() = 0;
    virtual void receiverTimedOut() = 0;
    virtual void receiverMotorReset() = 0;
    virtual void swiftrobotTimedOut() = 0;
    virtual void receiverConnected() = 0;
};

#endif