#pragma once

#include "pigpio.h"
#include "timer.hpp"

#define AUTONOMOUS_LED_GPIO 27
#define HEADLIGHT_RIGHT_LED_GPIO 13 // PWM
#define HEADLIGHT_LEFT_LED_GPIO 17 // PWM
#define BREAKING_LED_GPIO 16 // PWM
#define SIGNAL_LEFT_LED_GPIO 19
#define SIGNAL_RIGHT_LED_GPIO 12

#define ON 1
#define OFF 0

// brightnesses
#define BN_FULL 255
#define BN_DAYLIGHT 76 // 30 %

#define LATERAL_BLINK_INTERVAL 1000 // ms
#define SETUP_COMPLETE_BLINK_INTERVAL 300 // ms
#define TURNSIGNAL_BLINK_INTERVAL 500 // ms

//#define DEBUGGING
#ifdef DEBUGGING
#define DBG_PRINT(x...) printf(x)
#else
#define DBG_PRINT(x...) //
#endif

/**
 * abstracts control of the LEDs into higher level methods.
 * Methods which start with 'signal' play a pre defined sequence. Can be overwritten with a 'turnOff' method of the same LED
 * Methods which start with a 'turnOn' play a sequence or solid mode. These have to be turned off with a 'turnOff' method of the same LED
 * 
 */
class LEDController 
{
private:
  /// all possible modes for the led controller. Every color group uses only a subset
  enum Modes {
    off, autonomous, setupComplete, lateral, daylight, fullbeam, breaking, left_blink, right_blink, hazard
  };

// blue led group
  std::unique_ptr<Timer> lateralTimer;
  std::unique_ptr<Timer> setupCompleteTimer;
  int8_t setupCompleteCycleCnt = 0;
  bool setupCompleteRunning = false;
  std::function<void(void)> setupCompleteNext = nullptr;
// yellow led group
  /// timer for both signals
  std::unique_ptr<Timer> turnSignalTimer;
  Modes yellowMode = off;
  bool lastYellowState = OFF;

  bool lastBlueState = OFF;
  Modes blueMode = off;

  /// for both left and right
  uint8_t lastHeadlightBrightness = OFF;
  Modes whiteLeftMode = off;
  Modes whiteRightMode = off;
  std::function<void(void)> blinkingWhiteNext = nullptr;
  uint8_t lastBreakingLightBrightness = OFF;
  Modes redMode = off;
public:
  LEDController();
  ~LEDController();
  void turnOnAutonomous();
  void turnOnLateral();
  void turnOffAutonomous();
  void signalSetupComplete();
  void turnOnDaylight();
  void turnOffDaylight();
  void turnOnFullBeam();
  void turnOffFullBeam();
  void turnOnBreakingLight();
  void turnOffBreakingLights();
  void turnOnHazardLights();
  void turnOffHazardLights();
  void turnOnTurnSignalLeft();
  void turnOnTurnSignalRight();
  void turnOffTurnSignal();
private:
  void turnSignalCycle();
  void lateralCycle();
  void setupCompleteCycle();
};
