# Robocar Drivehub
This controls the motor, servo and LEDs and reads the remote control messages of the Robocar. It acts as an **intermediate** between actual hardware and Robocar iOS Application. Besides that it provides safety controls via a heart beat for remote control and iOS Application. If these connection time out, the car will automatically go into a fail safe mode depending on current mode. 

## Swiftrobot Topics:
The iOS Device has to be connected via a USB cable to the raspberry. For the communication SwiftRobot is used with the following topic usage.
### Subscribed
|Channel   	|Type   	| Description   	|
|---	    |---	|---	|
|0x0	    |interal_msg::UpdateMsg  	| Indicates if device is connected and/or application is running on iOS device. **Note:** not published by iOS device but swiftrobot_c|
|0x01   	|control_msg::Drive   	|Control commands for steering servo and motor.|

### Published
|Channel   	|Type   	| Description   	|
|---	    |---	|---	|
|0x11	    |base_msg::UInt32Array  	|Array with current hardware state. [mosfet temp, motor temp, motor rpm, battery voltage, wheelencoder ticks, wheelencoder ticks abs]|
|0x13   	|base_msg::UInt32Array   	|Array with remote control state. [throttle, steering, gear, lateral control on, autonomous on]. **Note:** Should not be used to control car by remote, since this is handled already by robocar_drivehub.|


## Modes
With a switcn on the remote control, the mode of Robocar can be switched.
- **Manual Control**: The car is completly controlled by the remote control. Throttle stick has to be in zero position when mode is activated.
- **Lateral Control**: Steering is controlled by the iOS Device and speed by the remote. In a timeout event car will go into manual control. 
- **Autonomous**(deadman switch): All controls of the car are given to the iOS Device. In a timeout event car will go into fail safe.

## LED Modes
The LEDs of the car are not only indicating driving states, but can also indicate software states (handled by robocar_drivehub).
- **Hazard Lights**: Fail Safe mode. Indicates that the remote control has no connection or the iOS Device is not connected when in autonomous mode.
- **Blue blinking**: Lateral control mode
- **Blue solid**: Autonomous mode

## Installation
swiftrobot_c and pigpio library are used as a dependency, so make sure it is installed.
For installation, use `make install` after building with `cmake` and `make`.


