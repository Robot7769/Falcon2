
#ifndef ODriveArduino_h
#define ODriveArduino_h

#include "Arduino.h"

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

class ODriveArduino {
public:
    enum AxisState {
        UNDEFINED = 0,           //<! will fall through to idle
        IDLE = 1,                //<! disable PWM and do nothing
        STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
        FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
        MOTOR_CALIBRATION = 4,   //<! run motor calibration
        SENSORLESS_CONTROL = 5,  //<! run sensorless control
        ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
        ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
        CLOSED_LOOP_CONTROL = 8  //<! run closed loop control
    };

    enum Error {
        NONE = 0x00,
        INVALID_STATE = 0x01, //<! an invalid state was requested
        DC_BUS_UNDER_VOLTAGE = 0x02,
        DC_BUS_OVER_VOLTAGE = 0x04,
        CURRENT_MEASUREMENT_TIMEOUT = 0x08,
        BRAKE_RESISTOR_DISARMED = 0x10, //<! the brake resistor was unexpectedly disarmed
        MOTOR_DISARMED = 0x20, //<! the motor was unexpectedly disarmed
        MOTOR_FAILED = 0x40, // Go to motor.hpp for information, check odrvX.axisX.motor.error for error value
        SENSORLESS_ESTIMATOR_FAILED = 0x80,
        ENCODER_FAILED = 0x100, // Go to encoder.hpp for information, check odrvX.axisX.encoder.error for error value
        CONTROLLER_FAILED = 0x200,
        POS_CTRL_DURING_SENSORLESS = 0x400,
        WATCHDOG_TIMER_EXPIRED = 0x800,
        READOUT = 0x1600
    };

    enum EncoderError {
        UNSTABLE_GAIN = 0x01,
        CPR_OUT_OF_RANGE = 0x02,
        NO_RESPONSE = 0x04,
        UNSUPPORTED_ENCODER_MODE = 0x08,
        ILLEGAL_HALL_STATE = 0x10,
        INDEX_NOT_FOUND_YET = 0x20,
    };

    enum MotorError {
        PHASE_RESISTANCE_OUT_OF_RANGE = 0x0001,
        PHASE_INDUCTANCE_OUT_OF_RANGE = 0x0002,
        ADC_FAILED = 0x0004,
        DRV_FAULT = 0x0008,
        CONTROL_DEADLINE_MISSED = 0x0010,
        NOT_IMPLEMENTED_MOTOR_TYPE = 0x0020,
        BRAKE_CURRENT_OUT_OF_RANGE = 0x0040,
        MODULATION_MAGNITUDE = 0x0080,
        BRAKE_DEADTIME_VIOLATION = 0x0100,
        UNEXPECTED_TIMER_CALLBACK = 0x0200,
        CURRENT_SENSE_SATURATION = 0x0400,
        INVERTER_OVER_TEMP = 0x0800
    };


    ODriveArduino(Stream& serial);

    void initializeMotors( bool full = false );
    float inputVoltage();
    void setState( int axis, AxisState state );
    AxisState getState( int axis );
    void turnOn();
    void turnOff();

    bool error();
    int error( int axis );
    int motorError( int axis );
    int encoderError( int axis );
    void resetError( int axis );
    void dumpErrors();

    float getPos( int axis );

    void move( int axis, float pos, float speed );
    void speed( int axis, float speed );
    void setAccel( float accel );

    template < typename T >
    void setProperty(String property, T val) {
        serial_ << "w " << property << " " << val << "\n";
        clearBuffer();
    }

    static String errorText( int error, int motor, int encoder );
private:
    String readString();
    String readLine( int timeout );
    void clearBuffer();

    Stream& serial_;
    int timeout_;
};

#endif //ODriveArduino_h
