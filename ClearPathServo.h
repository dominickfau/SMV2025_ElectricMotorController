#ifndef ClearPathServo_h
#define ClearPathServo_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

// These defs cause trouble on some versions of Arduino
#undef round

// Use the system yield() whenever possoible, since some platforms require it for housekeeping, especially
// ESP8266
#if (defined(ARDUINO) && ARDUINO >= 155) || defined(ESP8266)
#define YIELD yield();
#else
#define YIELD
#endif

/////////////////////////////////////////////////////////////////////
/// \class ClearPathServo ClearPathServo.h <ClearPathServo.h>
/// \brief Support for stepper motors with acceleration etc.
///
/// This defines a single 2 or 4 pin stepper motor, or stepper motor with fdriver chip, with optional
/// acceleration, deceleration, absolute positioning commands etc. Multiple
/// simultaneous steppers are supported, all moving
/// at different speeds and accelerations.
///
/// \par Operation
/// This module operates by computing a step time in microseconds. The step
/// time is recomputed after each step and after speed and acceleration
/// parameters are changed by the caller. The time of each step is recorded in
/// microseconds. The run() function steps the motor once if a new step is due.
/// The run() function must be called frequently until the motor is in the
/// desired position, after which time run() will do nothing.
///
/// \par Positioning
/// Positions are specified by a signed long integer. At
/// construction time, the current position of the motor is consider to be 0. Positive
/// positions are clockwise from the initial position; negative positions are
/// anticlockwise. The current position can be altered for instance after
/// initialization positioning.
///
/// \par Caveats
/// This is an open loop controller: If the motor stalls or is oversped,
/// ClearPathServo will not have a correct
/// idea of where the motor really is (since there is no feedback of the motor's
/// real position. We only know where we _think_ it is, relative to the
/// initial starting point).
///
/// \par Performance
/// The fastest motor speed that can be reliably supported is about 4000 steps per
/// second at a clock frequency of 16 MHz on Arduino such as Uno etc.
/// Faster processors can support faster stepping speeds.
/// However, any speed less than that
/// down to very slow speeds (much less than one per second) are also supported,
/// provided the run() function is called frequently enough to step the motor
/// whenever required for the speed set.
/// Calling setAcceleration() is expensive,
/// since it requires a square root to be calculated.
///
/// Gregor Christandl reports that with an Arduino Due and a simple test program,
/// he measured 43163 steps per second using runSpeed(),
/// and 16214 steps per second using run();
class ClearPathServo
{
public:
    /// Constructor. You can have multiple simultaneous steppers, all moving
    /// at different speeds and accelerations, provided you call their run()
    /// functions at frequent enough intervals. Current Position is set to 0, target
    /// position is set to 0. MaxSpeed and Acceleration default to 1.0.
    /// The motor pins will be initialised to OUTPUT mode during the
    /// constructor by a call to enableOutputs().
    /// \param[in] stepPin The GPIO pin number for the step signal
    /// \param[in] dirPin The GPIO pin number for the direction signal
    /// \param[in] enablePin The GPIO pin number for the enable signal
    ClearPathServo(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin);

    /// Checks if the motor is enabled
    /// @return true if the motor is enabled
    bool isEnabled();

    /// Set the target position. The run() function will try to move the motor (at most one step per call)
    /// from the current position to the target position set by the most
    /// recent call to this function. Caution: moveTo() also recalculates the speed for the next step.
    /// If you are trying to use constant speed movements, you should call setSpeed() after calling moveTo().
    /// \param[in] absolute The desired absolute position. Negative is
    /// anticlockwise from the 0 position.
    void moveTo(long absolute);

    /// Set the target position relative to the current position.
    /// \param[in] relative The desired position relative to the current position. Negative is
    /// anticlockwise from the current position.
    void move(long relative);

    /// Poll the motor and step it if a step is due, implementing
    /// accelerations and decelerations to achieve the target position. You must call this as
    /// frequently as possible, but at least once per minimum step time interval,
    /// preferably in your main loop. Note that each call to run() will make at most one step, and then only when a step is due,
    /// based on the current speed and the time since the last step.
    /// \return true if the motor is still running to the target position.
    boolean run();

    /// Poll the motor and step it if a step is due, implementing
    /// accelerations and decelerations to achieve the target position. You must call this as
    /// frequently as possible, but at least once per minimum step time interval,
    /// preferably in your main loop. Note that each call to run() will make at most one step, and then only when a step is due,
    /// based on the current speed and the time since the last step.
    /// \param[in] run If true the motor will continue to run, if false the motor will stop at the target position
    void runWhileButton(bool run);

    /// Poll the motor and step it if a step is due, implementing a constant
    /// speed as set by the most recent call to setSpeed(). You must call this as
    /// frequently as possible, but at least once per step interval,
    /// \return true if the motor was stepped.
    boolean runSpeed();

    /// Sets the maximum permitted speed. The run() function will accelerate
    /// up to the speed set by this function.
    /// Caution: the maximum speed achievable depends on your processor and clock speed.
    /// The default maxSpeed is 1.0 steps per second.
    /// \param[in] speed The desired maximum speed in steps per second. Must
    /// be > 0. Caution: Speeds that exceed the maximum speed supported by the processor may
    /// Result in non-linear accelerations and decelerations.
    void setMaxSpeed(float speed);

    /// Returns the maximum speed configured for this stepper
    /// that was previously set by setMaxSpeed();
    /// \return The currently configured maximum speed
    float maxSpeed();

    /// Sets the acceleration/deceleration rate.
    /// \param[in] acceleration The desired acceleration in steps per second
    /// per second. Must be > 0.0. This is an expensive call since it requires a square
    /// root to be calculated. Dont call more ofthen than needed
    void setAcceleration(float acceleration);

    /// Returns the acceleration/deceleration rate configured for this stepper
    /// that was previously set by setAcceleration();
    /// \return The currently configured acceleration/deceleration
    float acceleration();

    /// Sets the desired constant speed for use with runSpeed().
    /// \param[in] speed The desired constant speed in steps per
    /// second. Positive is clockwise. Speeds of more than 1000 steps per
    /// second are unreliable. Very slow speeds may be set (eg 0.00027777 for
    /// once per hour, approximately. Speed accuracy depends on the Arduino
    /// crystal. Jitter depends on how frequently you call the runSpeed() function.
    /// The speed will be limited by the current value of setMaxSpeed()
    void setSpeed(float speed);

    /// The most recently set speed.
    /// \return the most recent speed in steps per second
    float speed();

    /// The distance from the current position to the target position.
    /// \return the distance from the current position to the target position
    /// in steps. Positive is clockwise from the current position.
    long distanceToGo();

    /// The most recently set target position.
    /// \return the target position
    /// in steps. Positive is clockwise from the 0 position.
    long targetPosition();

    /// The current motor position.
    /// \return the current motor position
    /// in steps. Positive is clockwise from the 0 position.
    long currentPosition();

    /// Resets the current position of the motor, so that wherever the motor
    /// happens to be right now is considered to be the new 0 position. Useful
    /// for setting a zero position on a stepper after an initial hardware
    /// positioning move.
    /// Has the side effect of setting the current motor speed to 0.
    /// \param[in] position The position in steps of wherever the motor
    /// happens to be right now.
    void setCurrentPosition(long position);

    /// Moves the motor (with acceleration/deceleration)
    /// to the target position and blocks until it is at
    /// position. Dont use this in event loops, since it blocks.
    void runToPosition();

    /// Executes runSpeed() unless the targetPosition is reached.
    /// This function needs to be called often just like runSpeed() or run().
    /// Will step the motor if a step is required at the currently selected
    /// speed unless the target position has been reached.
    /// Does not implement accelerations.
    /// \return true if it stepped
    boolean runSpeedToPosition();

    /// Moves the motor (with acceleration/deceleration)
    /// to the new target position and blocks until it is at
    /// position. Dont use this in event loops, since it blocks.
    /// \param[in] position The new target position.
    void runToNewPosition(long position);

    /// Sets a new target position that causes the stepper
    /// to stop as quickly as possible, using the current speed and acceleration parameters.
    void stop();

    /// Disable motor holding torque.
    virtual void disableTorque();

    /// Enable motor holding torque.
    virtual void enableTorque();

    /// Sets the minimum pulse width allowed by the stepper driver. The minimum practical pulse width is
    /// approximately 20 microseconds. Times less than 20 microseconds
    /// will usually result in 20 microseconds or so.
    /// \param[in] minWidth The minimum pulse width in microseconds.
    void setMinPulseWidth(unsigned int minWidth);

    /// Sets the inversion for stepper driver pins
    /// \param[in] directionInvert True for inverted direction pin, false for non-inverted
    /// \param[in] stepInvert      True for inverted step pin, false for non-inverted
    /// \param[in] enableInvert    True for inverted enable pin, false (default) for non-inverted
    void setPinsInverted(bool directionInvert = false, bool stepInvert = false, bool enableInvert = false);

    /// Checks to see if the motor is currently running to a target
    /// \return true if the speed is not zero or not at the target position
    bool isRunning();

    /// Checks to see is the motor is currently accelerating
    /// \return true if the motor is running and the speed is not equal to the maximum speed
    bool isAccelerating();

    /// Checks to see is the motor is currently decelerating
    /// \return true if the motor is running and the speed is not equal to the minimum speed
    bool isDecelerating();

    /// Checks to see if the motor at the target velocity
    /// \return true if the speed is equal to the maximum speed
    bool isAtSpeed();

    /// Virtual destructor to prevent warnings during delete
    virtual ~ClearPathServo() {};

protected:
    /// \brief Direction indicator
    /// Symbolic names for the direction the motor is turning
    typedef enum
    {
        DIRECTION_CCW = 0, ///< Counter-Clockwise
        DIRECTION_CW = 1   ///< Clockwise
    } Direction;

    /// Forces the library to compute a new instantaneous speed and set that as
    /// the current speed. It is called by
    /// the library:
    /// \li  after each step
    /// \li  after change to maxSpeed through setMaxSpeed()
    /// \li  after change to acceleration through setAcceleration()
    /// \li  after change to target position (relative or absolute) through
    /// move() or moveTo()
    /// \return the new step interval
    virtual unsigned long computeNewSpeed();

    /// Low level function to set the motor output pins
    /// bit 0 of the mask corresponds to _pin[0]
    /// bit 1 of the mask corresponds to _pin[1]
    /// You can override this to implement, for example serial chip output insted of using the
    /// output pins directly
    virtual void setOutputPins(uint8_t mask);

    /// Called to execute a step. Only called when a new step is
    /// required. Subclasses may override to implement new stepping
    /// interfaces.
    virtual void step();

    /// Called to execute a clockwise(+) step. Only called when a new step is
    /// required. This increments the _currentPos and calls step()
    /// \return the updated current position
    long stepForward();

    /// Called to execute a counter-clockwise(-) step. Only called when a new step is
    /// required. This decrements the _currentPos and calls step()
    /// \return the updated current position
    long stepBackward();

    /// Current direction motor is spinning in
    /// Protected because some peoples subclasses need it to be so
    boolean _direction; // 1 == CW

    /// The current interval between steps in microseconds.
    /// 0 means the motor is currently stopped with _speed == 0
    unsigned long _stepInterval;

private:
    /// Arduino pin number assignments for the 2 or 4 pins required to interface to the
    /// stepper motor or driver
    uint8_t _pin[3];

    /// Whether the _pins is inverted or not
    uint8_t _pinInverted[3];

    /// Store motor enabled state
    bool _enabled;

    /// The current absolution position in steps.
    long _currentPos; // Steps
    long _auxCurrentPos;

    /// The target position in steps. The ClearPathServo library will move the
    /// motor from the _currentPos to the _targetPos, taking into account the
    /// max speed, acceleration and deceleration
    long _targetPos; // Steps

    /// The current motos speed in steps per second
    /// Positive is clockwise
    float _speed; // Steps per second

    /// The maximum permitted speed in steps per second. Must be > 0.
    float _maxSpeed;

    /// The acceleration to use to accelerate or decelerate the motor in steps
    /// per second per second. Must be > 0
    float _acceleration;
    float _sqrt_twoa; // Precomputed sqrt(2*_acceleration)

    /// The last step time in microseconds
    unsigned long _lastStepTime;

    /// The minimum allowed pulse width in microseconds
    unsigned int _minPulseWidth;

    /// The step counter for speed calculations
    long _n;

    /// Initial step size in microseconds
    float _c0;

    /// Last step size in microseconds
    float _cn;

    /// Min step size in microseconds based on maxSpeed
    float _cmin; // at max speed
};

#endif