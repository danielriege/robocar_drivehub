#include "pigpio.h"
#include "timer.hpp"

#define BLUE_LED_GPIO 21

#define ON 1
#define OFF 0

#define LANEKEEP_BLINK_INTERVAL 500
#define TURNSIGNAL_BLINK_INTERVAL 300

class LEDController 
{
public:
  std::unique_ptr<Timer> lanekeepBlinkingTimer;
  std::unique_ptr<Timer> turnSignalTimer;
public:
  /**
  Configures GPIOs which are used for LEDs. Should only be created once
  **/
  LEDController() {
//    if (gpioInitialise() < 0) {
//      printf("pigpio initialisation failed\n");
//      exit(1);
//    }

    lanekeepBlinkingTimer = std::make_unique<Timer>();
    turnSignalTimer = std::make_unique<Timer>();

    gpioSetMode(BLUE_LED_GPIO, PI_OUTPUT);
  }

  ~LEDController() {
//    gpioTerminate();
  }

  /**
  Autonomous mode on: Blue LEDs stay on
  **/
  void turnAutonomousOn() {
    gpioWrite(BLUE_LED_GPIO, ON);
  }

  void turnLanekeepOn() {
    lanekeepBlinkingTimer->setInterval(std::bind(&LEDController::lanekeepCycle, this), LANEKEEP_BLINK_INTERVAL);
  }

  /**
  autonomous mode off: Blue LEDs stay off
  **/
  void turnAutonomousOrLanekeepOff() {
    gpioWrite(BLUE_LED_GPIO, OFF);
  }
private:
  void lanekeepCycle() {
    if(gpioRead(BLUE_LED_GPIO) == ON) {
      gpioWrite(BLUE_LED_GPIO, OFF);
    } else {
      gpioWrite(BLUE_LED_GPIO, ON);
    }
  }
};
