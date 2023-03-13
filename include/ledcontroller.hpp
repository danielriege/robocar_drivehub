#include "pigpio.h"
#include "timer.hpp"

#define AUTONOMOUS_LED_GPIO 21
#define HEADLIGHT_LED_GPIO 13 // PWM
#define BREAKING_LED_GPIO 10 // PWM
#define SIGNAL_LEFT_LED_GPIO 16
#define SIGNAL_RIGHT_LED_GPIO 20 

#define ON 1
#define OFF 0

// brightnesses
#define BN_FULL 255
#define BN_DAYLIGHT 76 // 30 %

#define LATERAL_BLINK_INTERVAL 1000 // ms
#define SETUP_COMPLETE_BLINK_INTERVAL 300 // ms
#define TURNSIGNAL_BLINK_INTERVAL 800 // ms

/**
 * abstracts control of the LEDs into higher level methods.
 * Methods which start with 'signal' play a pre defined sequence. Can be overwritten with a 'turnOff' method of the same LED
 * Methods which start with a 'turnOn' play a sequence or solid mode. These have to be turned off with a 'turnOff' method of the same LED
 * 
 */
class LEDController 
{
private:
// blue led group
  std::unique_ptr<Timer> lateralTimer;
  std::unique_ptr<Timer> setupCompleteTimer;
  int8_t setupCompleteCycleCnt = 0;
  bool setupCompleteRunning = false;
  std::function<void(void)> setupCompleteNext = nullptr;
// yellow led group
  /// timer for both signals
  std::unique_ptr<Timer> turnSignalTimer;

  bool lastBlueState = OFF;
  uint8_t lastHeadlightBrightness = OFF;
  uint8_t lastBreakingLightBrightness = OFF;
public:
  /**
  * Configures GPIOs which are used for LEDs. Should only be created once
  **/
  LEDController() {
//    if (gpioInitialise() < 0) {
//      printf("pigpio initialisation failed\n");
//      exit(1);
//    }

    lateralTimer = std::make_unique<Timer>();
    setupCompleteTimer = std::make_unique<Timer>();
    turnSignalTimer = std::make_unique<Timer>();

    // configure GPIO as output
    gpioSetMode(AUTONOMOUS_LED_GPIO, PI_OUTPUT);
    gpioSetMode(HEADLIGHT_LED_GPIO, PI_OUTPUT);
    gpioSetMode(BREAKING_LED_GPIO, PI_OUTPUT);
    gpioSetMode(SIGNAL_LEFT_LED_GPIO, PI_OUTPUT);
    gpioSetMode(SIGNAL_RIGHT_LED_GPIO, PI_OUTPUT);
  }

  ~LEDController() {
//    gpioTerminate();
  }
  ///
  /// BLUE LEDS
  ///

  /**
  * Autonomous mode on: Blue LEDs stay on
  **/
  void turnOnAutonomous() {
    if (setupCompleteRunning) {
      setupCompleteNext = std::bind(&LEDController::turnOnAutonomous, this);
    } else {
      lateralTimer->stop();
      gpioWrite(AUTONOMOUS_LED_GPIO, ON);
      lastBlueState = ON;
    }
  }

  /**
   * Lateral Control mode on: Blue LEDs are blinking. Can only be overwritten
   * Does not update last state since temporary
   **/
  void turnOnLateral() {
    if (setupCompleteRunning) {
      setupCompleteNext = std::bind(&LEDController::turnOnLateral, this);
    } else {
      lateralTimer->setInterval(std::bind(&LEDController::lateralCycle, this), LATERAL_BLINK_INTERVAL);
    }
  }

  /**
  * autonomous mode off: Blue LEDs stay off
  **/
  void turnOffAutonomous() {
    if (setupCompleteRunning) {
      setupCompleteNext = std::bind(&LEDController::turnOffAutonomous, this);
    } else {
      lateralTimer->stop();
      gpioWrite(AUTONOMOUS_LED_GPIO, OFF);
      lastBlueState = OFF;
    }
  }

  /**
   * !!!Can not be overwritten by other method!!!
   * sequence of 3 on/off blue led cycles to show that the system has completed the setup. Goes back into last state where it started or goes into mode wich was called during sequence.
   * Does not update last state since temporary
   */
  void signalSetupComplete() {
    lateralTimer->stop();
    setupCompleteRunning = true;
    setupCompleteNext = nullptr;
    setupCompleteTimer->setInterval(std::bind(&LEDController::setupCompleteCycle, this), SETUP_COMPLETE_BLINK_INTERVAL);
  }

  ///
  /// WHITE AND RED LEDS
  ///

  /**
   * headlights and breaking lights in daylight mode
   */
  void turnOnDaylight() {
    gpioPWM(HEADLIGHT_LED_GPIO, BN_DAYLIGHT);
    lastHeadlightBrightness = BN_DAYLIGHT;
    gpioPWM(BREAKING_LED_GPIO, BN_DAYLIGHT);
    lastBreakingLightBrightness = BN_DAYLIGHT;
  }

  /**
   * headlights and breaking lights complety off
   */
  void turnOffDaylight() {
    gpioPWM(HEADLIGHT_LED_GPIO, OFF);
    lastHeadlightBrightness = OFF;
    gpioPWM(BREAKING_LED_GPIO, OFF);
    lastBreakingLightBrightness = OFF;
  }

  /**
   * headlights in full beam mode
   * Does not update last state since this is temporary
   */
  void turnOnFullBeam() {
    gpioPWM(HEADLIGHT_LED_GPIO, BN_FULL);
  }

    /**
   * headlights back into last state
   */
  void turnOffFullBeam() {
    gpioPWM(HEADLIGHT_LED_GPIO, lastHeadlightBrightness);
  }

  /**
   * breaking lights fully on
   * Does not update last state since this mode is temporary
   */
  void turnOnBreakingLight() {
    gpioPWM(BREAKING_LED_GPIO, BN_FULL);
  }

  /**
   * breaking lights off/back into daylight
   */
  void turnOffBreakingLights() {
    gpioPWM(BREAKING_LED_GPIO, lastBreakingLightBrightness);
  }

  ///
  /// YELLOW LEDS
  ///

private:

  void lateralCycle() {
    if(lastBlueState == ON) {
      gpioWrite(AUTONOMOUS_LED_GPIO, OFF);
      lastBlueState = OFF;
    } else {
      gpioWrite(AUTONOMOUS_LED_GPIO, ON);
      lastBlueState = ON;
    }
  }

  void setupCompleteCycle() {
    if(lastBlueState == ON) {
      gpioWrite(AUTONOMOUS_LED_GPIO, OFF);
      lastBlueState = OFF;
    } else {
      gpioWrite(AUTONOMOUS_LED_GPIO, ON);
      lastBlueState = ON;
    }
    setupCompleteCycleCnt++;
    if (setupCompleteCycleCnt >= 6) {
      setupCompleteTimer->stop();
      setupCompleteRunning = false;
      if (setupCompleteNext != nullptr) {
        setupCompleteNext();
      }
    }
  }
};
