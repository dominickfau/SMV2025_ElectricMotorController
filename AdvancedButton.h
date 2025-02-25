#pragma once

#include "Arduino.h"

class AdvancedButton
{
public:
    AdvancedButton(uint8_t pin, uint16_t debounce_ms = 50);
    void begin();
    void update();
    bool state();
    bool toggled();
    bool pressed();
    bool released();
    bool risingEdge();
    bool fallingEdge();
    bool held(uint16_t delay_ms);

    const static bool PRESSED = LOW;
    const static bool RELEASED = HIGH;

private:
    uint8_t _pin;
    uint16_t _debounceDelay;
    bool _lastButtonState;
    bool _buttonState;
    bool _reading;
    bool _risingEdge;
    bool _fallingEdge;
    bool _auxRising;
    bool _auxFalling;
    uint32_t _lastDebounceTime;
};