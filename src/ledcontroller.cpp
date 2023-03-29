 #include "ledcontroller.hpp"
 
 /**
  * Configures GPIOs which are used for LEDs. Should only be created once
  **/
  LEDController::LEDController() {
    if (gpioInitialise() < 0) {
      printf("pigpio initialisation failed\n");
      exit(1);
    }

    lateralTimer = std::make_unique<Timer>();
    setupCompleteTimer = std::make_unique<Timer>();
    turnSignalTimer = std::make_unique<Timer>();

    // configure GPIO as output
    gpioSetMode(AUTONOMOUS_LED_GPIO, PI_OUTPUT);
    gpioSetMode(HEADLIGHT_LEFT_LED_GPIO, PI_OUTPUT);
    gpioSetMode(HEADLIGHT_RIGHT_LED_GPIO, PI_OUTPUT);
    gpioSetMode(BREAKING_LED_GPIO, PI_OUTPUT);
    gpioSetMode(SIGNAL_LEFT_LED_GPIO, PI_OUTPUT);
    gpioSetMode(SIGNAL_RIGHT_LED_GPIO, PI_OUTPUT);
  }

  LEDController::~LEDController() {
    gpioTerminate();
  }
  ///
  /// BLUE LEDS
  ///

  /**
  * Autonomous mode on: Blue LEDs stay on
  **/
  void LEDController::turnOnAutonomous() {
    if (setupCompleteRunning) {
      setupCompleteNext = std::bind(&LEDController::turnOnAutonomous, this);
    } else {
      if (blueMode != autonomous) {
        blueMode = autonomous;
        lateralTimer->stop();
        gpioWrite(AUTONOMOUS_LED_GPIO, ON);
        lastBlueState = ON;
      }
    }
  }

  /**
   * Lateral Control mode on: Blue LEDs are blinking. Can only be overwritten
   * Does not update last state since temporary
   **/
  void LEDController::turnOnLateral() {
    if (setupCompleteRunning) {
      setupCompleteNext = std::bind(&LEDController::turnOnLateral, this);
    } else {
      if (blueMode != lateral) {
        lateralCycle(); // first start manual so there is a change right away
        blueMode = lateral;
        lateralTimer->setInterval(std::bind(&LEDController::lateralCycle, this), LATERAL_BLINK_INTERVAL);
      }
    }
  }

  /**
  * autonomous mode off: Blue LEDs stay off
  **/
  void LEDController::turnOffAutonomous() {
    if (setupCompleteRunning) {
      setupCompleteNext = std::bind(&LEDController::turnOffAutonomous, this);
    } else {
      if (blueMode != off) {
        blueMode = off;
        lateralTimer->stop();
        gpioWrite(AUTONOMOUS_LED_GPIO, OFF);
        lastBlueState = OFF;
      }
    }
  }

  /**
   * !!!Can not be overwritten by other method!!!
   * sequence of 3 on/off blue led cycles to show that the system has completed the setup. Goes back into last state where it started or goes into mode wich was called during sequence.
   * Does not update last state since temporary
   */
  void LEDController::signalSetupComplete() {
    if (blueMode != setupComplete) {
      blueMode = setupComplete;
      lateralTimer->stop();
      setupCompleteRunning = true;
      setupCompleteNext = nullptr;
      setupCompleteTimer->setInterval(std::bind(&LEDController::setupCompleteCycle, this), SETUP_COMPLETE_BLINK_INTERVAL);
    }
  }

  ///
  /// WHITE AND RED LEDS
  ///

  /**
   * headlights and breaking lights in daylight mode
   * This method can be called again if one blinking light is active
   */
  void LEDController::turnOnDaylight() {
    if (yellowMode == hazard || yellowMode == left_blink) {
      blinkingWhiteNext = std::bind(&LEDController::turnOnDaylight, this);
    } else {
      if (whiteLeftMode != daylight) {
        whiteLeftMode = daylight;
        gpioPWM(HEADLIGHT_LEFT_LED_GPIO, BN_DAYLIGHT);
      }
    }

    if (yellowMode == hazard || yellowMode == right_blink) {
      blinkingWhiteNext = std::bind(&LEDController::turnOnDaylight, this);
    } else {
      if (whiteRightMode != daylight) {
        whiteRightMode = daylight;
        gpioPWM(HEADLIGHT_RIGHT_LED_GPIO, BN_DAYLIGHT);
      }
    }
    lastHeadlightBrightness = BN_DAYLIGHT;

    if (redMode != daylight) {
      redMode = daylight;
      gpioPWM(BREAKING_LED_GPIO, BN_DAYLIGHT);
      lastBreakingLightBrightness = BN_DAYLIGHT;
    }
  }

  /**
   * headlights and breaking lights complety off
   */
  void LEDController::turnOffDaylight() {
    blinkingWhiteNext = nullptr;
    if (whiteLeftMode != off || whiteRightMode != off) {
      whiteLeftMode = off;
      whiteRightMode = off;
      gpioPWM(HEADLIGHT_RIGHT_LED_GPIO, OFF);
      gpioPWM(HEADLIGHT_LEFT_LED_GPIO, OFF);
      lastHeadlightBrightness = OFF;
    }
    if (redMode != off) {
      redMode = off;
      gpioPWM(BREAKING_LED_GPIO, OFF);
      lastBreakingLightBrightness = OFF;
    }
  }

  /**
   * headlights in full beam mode
   * Does not update last state since this is temporary
   */
  void LEDController::turnOnFullBeam() {
    if (yellowMode == hazard || yellowMode == left_blink) {
      blinkingWhiteNext = std::bind(&LEDController::turnOnFullBeam, this);
    } else {
      gpioPWM(HEADLIGHT_LEFT_LED_GPIO, BN_FULL);
    }

    if (yellowMode == hazard || yellowMode == right_blink) {
      blinkingWhiteNext = std::bind(&LEDController::turnOnFullBeam, this);
    } else {
      gpioPWM(HEADLIGHT_RIGHT_LED_GPIO, BN_FULL);
    }
    lastHeadlightBrightness = BN_FULL;
  }

    /**
   * headlights back into last state
   */
  void LEDController::turnOffFullBeam() {
    if (lastHeadlightBrightness == OFF) {
      blinkingWhiteNext = nullptr;
      gpioPWM(HEADLIGHT_LEFT_LED_GPIO, OFF);
      gpioPWM(HEADLIGHT_RIGHT_LED_GPIO, OFF);
    } else if (lastHeadlightBrightness == BN_DAYLIGHT) {
      blinkingWhiteNext = std::bind(&LEDController::turnOnDaylight, this);
    }
  }

  /**
   * breaking lights fully on
   * Does not update last state since this mode is temporary
   */
  void LEDController::turnOnBreakingLight() {
    gpioPWM(BREAKING_LED_GPIO, BN_FULL);
  }

  /**
   * breaking lights off/back into daylight
   */
  void LEDController::turnOffBreakingLights() {
    gpioPWM(BREAKING_LED_GPIO, lastBreakingLightBrightness);
  }

  ///
  /// YELLOW LEDS
  ///

  void LEDController::turnOnHazardLights() {
    if (yellowMode == off) {
      yellowMode = hazard;
      gpioPWM(HEADLIGHT_RIGHT_LED_GPIO, OFF);
      gpioPWM(HEADLIGHT_LEFT_LED_GPIO, OFF);
      turnSignalCycle();
      turnSignalTimer->setInterval(std::bind(&LEDController::turnSignalCycle, this), TURNSIGNAL_BLINK_INTERVAL);
    }
    yellowMode = hazard;
  }

  void LEDController::turnOffHazardLights() {
    turnSignalTimer->stop();
    gpioWrite(SIGNAL_LEFT_LED_GPIO, OFF);
    gpioWrite(SIGNAL_RIGHT_LED_GPIO, OFF);
    gpioPWM(HEADLIGHT_RIGHT_LED_GPIO, lastHeadlightBrightness);
    gpioPWM(HEADLIGHT_LEFT_LED_GPIO, lastHeadlightBrightness);
    yellowMode = off;
  }

  void LEDController::turnOnTurnSignalLeft() {
    if (yellowMode == off) {
      yellowMode = left_blink;
      gpioPWM(HEADLIGHT_LEFT_LED_GPIO, OFF);
      turnSignalCycle();
      turnSignalTimer->setInterval(std::bind(&LEDController::turnSignalCycle, this), TURNSIGNAL_BLINK_INTERVAL);
    }
    yellowMode = left_blink;
  }

  void LEDController::turnOnTurnSignalRight() {
    if (yellowMode == off) {
      yellowMode = right_blink;
      gpioPWM(HEADLIGHT_RIGHT_LED_GPIO, OFF);
      turnSignalCycle();
      turnSignalTimer->setInterval(std::bind(&LEDController::turnSignalCycle, this), TURNSIGNAL_BLINK_INTERVAL);
    }
    yellowMode = right_blink;
  }

  void LEDController::turnOffTurnSignal() {
    turnSignalTimer->stop();
    gpioWrite(SIGNAL_LEFT_LED_GPIO, OFF);
    gpioWrite(SIGNAL_RIGHT_LED_GPIO, OFF);
    gpioPWM(HEADLIGHT_RIGHT_LED_GPIO, lastHeadlightBrightness);
    gpioPWM(HEADLIGHT_LEFT_LED_GPIO, lastHeadlightBrightness);
    yellowMode = off;
  }

  void LEDController::turnSignalCycle() {
    if (yellowMode == hazard) {
      if (lastYellowState == ON) {
        gpioWrite(SIGNAL_LEFT_LED_GPIO, OFF);
        gpioWrite(SIGNAL_RIGHT_LED_GPIO, OFF);
        lastYellowState = OFF;
      } else {
        gpioWrite(SIGNAL_LEFT_LED_GPIO, ON);
        gpioWrite(SIGNAL_RIGHT_LED_GPIO, ON);
        lastYellowState = ON;
       }
    } else if (yellowMode == left_blink) {
      if (lastYellowState == ON) {
        gpioWrite(SIGNAL_LEFT_LED_GPIO, OFF);
        lastYellowState = OFF;
      } else {
        gpioWrite(SIGNAL_LEFT_LED_GPIO, ON);
        lastYellowState = ON;
      }
    } else if (yellowMode == right_blink) {
      if (lastYellowState == ON) {
        gpioWrite(SIGNAL_RIGHT_LED_GPIO, OFF);
        lastYellowState = OFF;
      } else {
        gpioWrite(SIGNAL_RIGHT_LED_GPIO, ON);
        lastYellowState = ON;
      }
    }
  }

  void LEDController::lateralCycle() {
    if(lastBlueState == ON) {
      gpioWrite(AUTONOMOUS_LED_GPIO, OFF);
      lastBlueState = OFF;
    } else {
      gpioWrite(AUTONOMOUS_LED_GPIO, ON);
      lastBlueState = ON;
    }
  }

  void LEDController::setupCompleteCycle() {
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
      printf("setup completed\n");
      setupCompleteRunning = false;
      if (setupCompleteNext != nullptr) {
        printf("starting next\n");
        setupCompleteNext();
      }
    }
  }