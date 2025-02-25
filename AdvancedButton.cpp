#include <Arduino.h>
#include "AdvancedButton.h"

AdvancedButton::AdvancedButton(uint8_t pin, uint16_t debounce_ms)
{
  _pin = pin;
  _debounceDelay = debounce_ms;
  _lastButtonState = RELEASED;
  _buttonState = RELEASED;
  _reading = RELEASED;
  _risingEdge = false;
  _fallingEdge = false;
  _auxRising = false;
  _auxFalling = false;
  _lastDebounceTime = 0;
}

void AdvancedButton::begin()
{
  pinMode(_pin, INPUT_PULLUP);
}

//
// public methods
//

void AdvancedButton::update()
{
  _reading = digitalRead(_pin);
  _risingEdge = false;
  _fallingEdge = false;

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (_reading != _lastButtonState)
  {
    // reset the debouncing timer
    _lastDebounceTime = millis();
  }

  if ((millis() - _lastDebounceTime) > _debounceDelay)
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (_reading != _buttonState)
    {
      _buttonState = _reading;

      // Rising
      if (_buttonState == PRESSED && !_auxRising)
      {
        _auxRising = true;
        _auxFalling = false;
        _risingEdge = true;
      }

      // Falling
      if (_buttonState == RELEASED && !_auxFalling)
      {
        _auxRising = false;
        _auxFalling = true;
        _fallingEdge = true;
      }
    }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  _lastButtonState = _reading;
}

// Returns the current button state
bool AdvancedButton::state()
{
  return _buttonState;
}

// Has the button been toggled from on -> off, or vice versa
bool AdvancedButton::toggled()
{
  return _risingEdge || _fallingEdge;
}

bool AdvancedButton::pressed()
{
  return this->state() == PRESSED;
}

bool AdvancedButton::released()
{
  return this->state() == RELEASED;
}

// State changed from Released -> Pressed
bool AdvancedButton::risingEdge()
{
  return _risingEdge;
}

// State changed from Pressed -> Released
bool AdvancedButton::fallingEdge()
{
  return _fallingEdge;
}

// Has the button been pressed for some time.
bool AdvancedButton::held(uint16_t delay_ms)
{
  static unsigned long time = millis();

  // Check if button is pressed /held "HIGH" and reset is false
  if (this->toggled())
  {
    time = millis();
  }

  return this->pressed() && (millis() - time > delay_ms);
}