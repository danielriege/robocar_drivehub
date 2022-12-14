#ifndef RECEIVER_DATA_HPP
#define RECEIVER_DATA_HPP

#define THROTTLE_CHANNEL 1
#define STEERING_CHANNEL 2
#define GEAR_CHANNEL 3
#define AUTONOMOUS_CHANNEL 4

enum Gear {
    undefined, drive, reverse
};

struct ReceiverData {
    /// in vesc friendly range
    float throttle = 0;
    /// in vesc friendly range
    float steering = 0;
    Gear gearSelector = undefined;
    bool autonomous = 0;
    bool autonomous_deadman = 0;
};

#endif
