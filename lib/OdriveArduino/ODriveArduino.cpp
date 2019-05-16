
#include "Arduino.h"
#include "ODriveArduino.h"
#include <cstdlib>

static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

bool strToFloat( const String& str, float& res ) {
    char *end;
    res = strtod( str.c_str(), &end );
    return str.c_str() != end;
}

bool strToInt( const String& str, int& res ) {
    char *end;
    res = strtol( str.c_str(), &end, 10 );
    return str.c_str() != end;
}

String errorStr( int err ) {
    switch ( err ) {
        case ODriveArduino::NONE: return "NONE";
        case ODriveArduino::INVALID_STATE: return "INVALID_STATE";
        case ODriveArduino::DC_BUS_UNDER_VOLTAGE: return "DC_BUS_UNDER_VOLTAGE";
        case ODriveArduino::DC_BUS_OVER_VOLTAGE: return "DC_BUS_OVER_VOLTAGE";
        case ODriveArduino::CURRENT_MEASUREMENT_TIMEOUT: return "CURRENT_MEASUREMENT_TIMEOUT";
        case ODriveArduino::BRAKE_RESISTOR_DISARMED: return "BRAKE_RESISTOR_DISARMED";
        case ODriveArduino::MOTOR_DISARMED: return "MOTOR_DISARMED";
        case ODriveArduino::MOTOR_FAILED: return "MOTOR_FAILED";
        case ODriveArduino::SENSORLESS_ESTIMATOR_FAILED: return "SENSORLESS_ESTIMATOR_FAILED";
        case ODriveArduino::ENCODER_FAILED: return "ENCODER_FAILED";
        case ODriveArduino::CONTROLLER_FAILED: return "CONTROLLER_FAILED";
        case ODriveArduino::POS_CTRL_DURING_SENSORLESS: return "PSO_CTRL_DURING_SENSORLESS";
        case ODriveArduino::WATCHDOG_TIMER_EXPIRED: return "WATCHDOG_TIMER_EXPIRED";
        case ODriveArduino::READOUT: return "READOUT";
    }
}

String encoderErrorStr( int err ) {
    switch ( err ) {
        case ODriveArduino::NONE: return "NONE";
        case ODriveArduino::UNSTABLE_GAIN: return "UNSTABLE_GAIN";
        case ODriveArduino::CPR_OUT_OF_RANGE: return "CPR_OUT_OF_RANGE";
        case ODriveArduino::NO_RESPONSE: return "NO_RESPONSE";
        case ODriveArduino::UNSUPPORTED_ENCODER_MODE: return "UNSUPPORTED_ENCODER_MODE";
        case ODriveArduino::ILLEGAL_HALL_STATE: return "ILLEGAL_HALL_STATE";
        case ODriveArduino::INDEX_NOT_FOUND_YET: return "INDEX_NOT_FOUND_YET";
    }
}

String motorErrorStr( int err ) {
    switch( err ) {
        case ODriveArduino::NONE: return "NONE";
        case ODriveArduino::PHASE_RESISTANCE_OUT_OF_RANGE: return "PHASE_RESISTANCE_OUT_OF_RANGE";
        case ODriveArduino::PHASE_INDUCTANCE_OUT_OF_RANGE: return "PHASE_INDUCTANCE_OUT_OF_RANGE";
        case ODriveArduino::ADC_FAILED: return "ADC_FAILED";
        case ODriveArduino::DRV_FAULT: return "DRV_FAULT";
        case ODriveArduino::CONTROL_DEADLINE_MISSED: return "CONTROL_DEADLINE_MISSED";
        case ODriveArduino::NOT_IMPLEMENTED_MOTOR_TYPE: return "NOT_IMPLEMENTED_MOTOR_TYPE";
        case ODriveArduino::BRAKE_CURRENT_OUT_OF_RANGE: return "BRAKE_CURRENT_OUT_OF_RANGE";
        case ODriveArduino::MODULATION_MAGNITUDE: return "MODULATION_MAGNITUDE";
        case ODriveArduino::BRAKE_DEADTIME_VIOLATION: return "BRAKE_DEADTIME_VIOLATION";
        case ODriveArduino::UNEXPECTED_TIMER_CALLBACK: return "UNEXPECTED_TIMER_CALLBACK";
        case ODriveArduino::CURRENT_SENSE_SATURATION: return "CURRENT_SENSE_SATURATION";
        case ODriveArduino::INVERTER_OVER_TEMP: return "INVERTER_OVER_TEMP";
    }
}

ODriveArduino::ODriveArduino(Stream& serial)
    : serial_(serial), timeout_(500)
{}

String ODriveArduino::readLine( int timeout ) {
    auto start = millis();
    String ret;
    while ( millis() < start + timeout ) {
        if ( !serial_.available() )
            continue;
        char c = serial_.read();
        if ( c == '\n' )
            return ret;
        ret += c;
    }
    return "";
}


void ODriveArduino::initializeMotors( bool full ) {
    for ( int axis : { 0, 1 } ) {
        resetError( axis );
        if ( full )
            setState( axis, AxisState::FULL_CALIBRATION_SEQUENCE );
        else
            setState( axis, AxisState::ENCODER_OFFSET_CALIBRATION );
    }
    while ( getState( 0 ) != AxisState::IDLE || getState( 1 ) != AxisState::IDLE ) {
        if (error(0) || error(1))
            dumpErrors();
        delay( 1 );
    }
}

float ODriveArduino::inputVoltage() {
    clearBuffer();
    serial_ << "r vbus_voltage\n";
    String response = readLine( timeout_ );
    float voltage;
    if ( strToFloat( response, voltage ) )
        return voltage;
    return -1;
}

void ODriveArduino::clearBuffer() {
    while ( serial_.available() )
        serial_.read();
}

void ODriveArduino::setState( int axis, ODriveArduino::AxisState state ) {
    assert( axis == 0 || axis == 1 );
    clearBuffer();
    serial_ << "w axis" << axis << ".requested_state " << state << "\n";
}

ODriveArduino::AxisState ODriveArduino::getState( int axis ) {
    clearBuffer();
    assert( axis == 0 || axis == 1 );
    serial_ << "r axis" << axis << ".current_state\n";
    String response = readLine( timeout_ );
    int state;
    if ( strToInt( response, state ) )
        return static_cast< AxisState >( state );
    return AxisState::UNDEFINED;
}

int ODriveArduino::error( int axis ) {
    assert( axis == 0 || axis == 1 );
    serial_ << "r axis" << axis << ".error\n";
    String response = readLine( timeout_ );
    int error;
    if ( strToInt( response, error ) )
        return error;
    return Error::READOUT;
}

bool ODriveArduino::error() {
    return error( 0 ) || error( 1 );
}

void ODriveArduino::dumpErrors() {
    for ( int axis : { 0, 1 } ) {
        if ( !error( axis ) )
            continue;
        int err = error( axis );
        int motor = motorError( axis );
        int encoder = encoderError( axis );
        Serial.print("Error axis");
        Serial.print(axis);
        Serial.print(": ");
        Serial.println( errorText( err, motor, encoder ) );
    }
}

int ODriveArduino::motorError( int axis ) {
    assert( axis == 0 || axis == 1 );
    serial_ << "r axis" << axis << ".motor.error\n";
    String response = readLine( timeout_ );
    int error;
    if ( strToInt( response, error ) )
        return error;
    return Error::READOUT;
}

int ODriveArduino::encoderError( int axis ) {
    assert( axis == 0 || axis == 1 );
    serial_ << "r axis" << axis << ".encoder.error\n";
    String response = readLine( timeout_ );
    int error;
    if ( strToInt( response, error ) )
        return error;
    return Error::READOUT;
}

void ODriveArduino::resetError( int axis ) {
    assert( axis == 0 || axis == 1 );
    serial_ << "w axis" << axis << ".error 0\n";
    serial_ << "w axis" << axis << ".motor.error 0 \n";
    serial_ << "w axis" << axis << ".encoder.error 0 \n";
}

String ODriveArduino::errorText( int error, int motor, int encoder ) {
    if ( error == 0 )
        return "No error";
    String res;
    String separator;
    for ( int i = 0; i != 32; i++ ) {
        int mask = 1 << i;
        if ( error & mask ) {
            res += separator + errorStr( mask );
            separator = ", ";
        }
    }
    for ( int i = 0; i != 32; i++ ) {
        int mask = 1 << i;
        if ( motor & mask ) {
            res += separator + motorErrorStr( mask );
            separator = ", ";
        }
    }
    for ( int i = 0; i != 32; i++ ) {
        int mask = 1 << i;
        if ( encoder & mask ) {
            res += separator + encoderErrorStr( mask );
            separator = ", ";
        }
    }
    return res;
}


void ODriveArduino::turnOn() {
    for ( int axis : { 0, 1 } ) {
        setState( axis, AxisState::CLOSED_LOOP_CONTROL );
    }

}

void ODriveArduino::turnOff() {
    for ( int axis : { 0, 1 } ) {
        setState( axis, AxisState::IDLE );
    }
}

float ODriveArduino::getPos( int axis ) {
    assert( axis == 0 || axis == 1 );
    serial_ << "r axis" << axis << ".encoder.pos_estimate\n";
    String response = readLine( timeout_ );
    float pos;
    strToFloat( response, pos );
    return pos;
}

void ODriveArduino::move( int axis, float pos, float speed ) {
    assert( axis == 0 || axis == 1 );
    serial_ << "w axis" <<  axis << ".trap_traj.config.vel_limit " << speed << "\n";
    serial_ << "t " << axis << " " << pos << "\n";
}

void ODriveArduino::setAccel( float accel ) {
    clearBuffer();
    for ( int axis : { 0, 1 } ) {
        serial_ << "w axis" <<  axis << ".trap_traj.config.accel_limit " << accel << "\n";
        serial_ << "w axis" <<  axis << ".trap_traj.config.decel_limit " << accel << "\n";
    }
}

void ODriveArduino::speed( int axis, float speed ) {
    serial_ << "v " << axis << " " << speed << "\n";
}

