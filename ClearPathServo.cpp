// ClearPathServo.cpp

#include "ClearPathServo.h"

void ClearPathServo::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
        _targetPos = absolute;
        computeNewSpeed();
        // compute new n?
    }
}

void ClearPathServo::move(long relative)
{
    moveTo(_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean ClearPathServo::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval)
        return false;

    unsigned long time = micros();
    if (time - _lastStepTime >= _stepInterval)
    {
        if (_direction == DIRECTION_CW)
        {
            // Clockwise
            _currentPos += 1;
        }
        else
        {
            // Anticlockwise
            _currentPos -= 1;
        }
        step();

        _lastStepTime = time; // Caution: does not account for costs in step()

        return true;
    }
    else
    {
        return false;
    }
}

long ClearPathServo::distanceToGo()
{
    return _targetPos - _currentPos;
}

long ClearPathServo::targetPosition()
{
    return _targetPos;
}

long ClearPathServo::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void ClearPathServo::setCurrentPosition(long position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
    _speed = 0.0;
}

// Subclasses can override
unsigned long ClearPathServo::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
        // We are at the target and its time to stop
        _stepInterval = 0;
        _speed = 0.0;
        _n = 0;
        return _stepInterval;
    }

    if (distanceTo > 0)
    {
        // We are anticlockwise from the target
        // Need to go clockwise from here, maybe decelerate now
        if (_n > 0)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
                _n = -stepsToStop; // Start deceleration
        }
        else if (_n < 0)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
                _n = -_n; // Start accceleration
        }
    }
    else if (distanceTo < 0)
    {
        // We are clockwise from the target
        // Need to go anticlockwise from here, maybe decelerate
        if (_n > 0)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
                _n = -stepsToStop; // Start deceleration
        }
        else if (_n < 0)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
                _n = -_n; // Start accceleration
        }
    }

    // Need to accelerate or decelerate
    if (_n == 0)
    {
        // First step from stopped
        _cn = _c0;
        _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
        // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
        _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
        _cn = max(_cn, _cmin);
    }
    _n++;
    _stepInterval = _cn;
    _speed = 1000000.0 / _cn;
    if (_direction == DIRECTION_CCW)
        _speed = -_speed;

    return _stepInterval;
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
boolean ClearPathServo::run()
{
    if (runSpeed())
        computeNewSpeed();
    return _speed != 0.0 || distanceToGo() != 0;
}

void ClearPathServo::runWhileButton(bool run)
{
    bool first = run && !isRunning();

    if (run && isAccelerating())
    {
        _auxCurrentPos = _currentPos;
    }

    // If this is the first time we are running, we need to reset current position and set a target off in the distance
    if (first)
    {
        setCurrentPosition(0);
        long stepsToDo = (long)((_maxSpeed * _maxSpeed) / (2.0 * _acceleration)); // Equation 16
        moveTo(stepsToDo + stepsToDo + stepsToDo);
    }

    if (runSpeed())
    {
        computeNewSpeed();
        if (isDecelerating())
        {
            _auxCurrentPos = 0;
        }
    }

    if (run && !isAccelerating() && (_auxCurrentPos > 0))
    {
        _currentPos = _auxCurrentPos;
    }
}

ClearPathServo::ClearPathServo(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin)
{
    _currentPos = 0;
    _auxCurrentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 0.0;
    _acceleration = 0.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _pin[0] = stepPin;
    _pin[1] = dirPin;
    _pin[2] = enablePin;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    int i;
    for (i = 0; i < 4; i++)
        _pinInverted[i] = 0;

    // Some reasonable default
    setAcceleration(1);
    setMaxSpeed(1);
}

void ClearPathServo::setMaxSpeed(float speed)
{
    if (speed < 0.0)
        speed = -speed;
    if (_maxSpeed != speed)
    {
        _maxSpeed = speed;
        _cmin = 1000000.0 / speed;
        // Recompute _n from current speed and adjust speed if accelerating or cruising
        if (_n > 0)
        {
            _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
            computeNewSpeed();
        }
    }
}

float ClearPathServo::maxSpeed()
{
    return _maxSpeed;
}

void ClearPathServo::setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
        return;

    if (acceleration < 0.0)
        acceleration = -acceleration;
    if (_acceleration != acceleration)
    {
        // Recompute _n per Equation 17
        _n = _n * (_acceleration / acceleration);
        // New c0 per Equation 7, with correction per Equation 15
        _c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
        _acceleration = acceleration;
        computeNewSpeed();
    }
}

float ClearPathServo::acceleration()
{
    return _acceleration;
}

void ClearPathServo::setSpeed(float speed)
{
    if (speed == _speed)
        return;
    speed = constrain(speed, -_maxSpeed, _maxSpeed);
    if (speed == 0.0)
        _stepInterval = 0;
    else
    {
        _stepInterval = fabs(1000000.0 / speed);
        _direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    _speed = speed;
}

float ClearPathServo::speed()
{
    return _speed;
}

// Subclasses can override
void ClearPathServo::step()
{
    // _pin[0] is step, _pin[1] is direction
    setOutputPins(_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
    setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
    // Caution 200ns setup time
    // Delay the minimum allowed pulse width
    delayMicroseconds(_minPulseWidth);
    setOutputPins(_direction ? 0b10 : 0b00); // step LOW
}

long ClearPathServo::stepForward()
{
    // Clockwise
    _currentPos += 1;
    step();
    _lastStepTime = micros();
    return _currentPos;
}

long ClearPathServo::stepBackward()
{
    // Counter-clockwise
    _currentPos -= 1;
    step();
    _lastStepTime = micros();
    return _currentPos;
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void ClearPathServo::setOutputPins(uint8_t mask)
{
    uint8_t numpins = 2;
    uint8_t i;
    for (i = 0; i < numpins; i++)
        digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
}

bool ClearPathServo::isEnabled()
{
    return _enabled;
}

void ClearPathServo::disableTorque()
{
    setOutputPins(0); // Handles inversion automatically
    digitalWrite(_pin[2], LOW ^ _pinInverted[2]);
    _enabled = false;
}

void ClearPathServo::enableTorque()
{
    // Handles inversion automatically
    // Enable is active high
    digitalWrite(_pin[2], HIGH ^ _pinInverted[2]);
    _enabled = true;
}

void ClearPathServo::setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}

void ClearPathServo::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert)
{
    _pinInverted[0] = stepInvert;
    _pinInverted[1] = directionInvert;
    _pinInverted[2] = enableInvert;
}

// Blocks until the target position is reached and stopped
void ClearPathServo::runToPosition()
{
    while (run())
        YIELD; // Let system housekeeping occur
}

boolean ClearPathServo::runSpeedToPosition()
{
    if (_targetPos == _currentPos)
        return false;
    if (_targetPos > _currentPos)
        _direction = DIRECTION_CW;
    else
        _direction = DIRECTION_CCW;
    return runSpeed();
}

// Blocks until the new target position is reached
void ClearPathServo::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

void ClearPathServo::stop()
{
    if (_speed != 0.0)
    {
        long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
        if (_speed > 0)
            move(stepsToStop);
        else
            move(-stepsToStop);
    }
}

bool ClearPathServo::isRunning()
{
    return !(_speed == 0.0 && _targetPos == _currentPos);
}

bool ClearPathServo::isAccelerating()
{
    return isRunning() && _n > 0 && !isAtSpeed();
}

bool ClearPathServo::isDecelerating()
{
    return isRunning() && _n < 0;
}

bool ClearPathServo::isAtSpeed()
{
    return isRunning() && _speed == (_speed > 0 ? _maxSpeed : -_maxSpeed);
}