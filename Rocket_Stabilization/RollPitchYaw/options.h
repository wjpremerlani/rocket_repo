#define ROLL_ONLY 1
#define ROLL_PLUS_VERTICAL 2
#define ROLL_PLUS_TILT 3
#define TILT_PATTERN 4
// OUTPUT_HZ must be either 10 or 25
//#define OUTPUT_HZ 10
#define OUTPUT_HZ 25

#define CONTROL_TYPE ROLL_PLUS_VERTICAL

// Uncomment the line below if you wish to use gain scheduling,
// and define the gains schedule that you want in gain_defs.h.
// Gain scheduling is separate from tilt patterns. You can use neither, either or both features.
//#define GAIN_SCHEDULING

#if ( CONTROL_TYPE == ROLL_ONLY )
#define CONTROL_TEXT "roll only"
#define ROLL_ENABLE 1
#define YAW_PITCH_ENABLE 0
#define TILT_X 0
#define TILT_Y 0
#define TILT_Z 16384
#endif // ROLL_ONLY

#if ( CONTROL_TYPE == ROLL_PLUS_VERTICAL )
#define CONTROL_TEXT "roll and vertical"
#define ROLL_ENABLE 1
#define YAW_PITCH_ENABLE 1
#define TILT_X 0
#define TILT_Y 0
#define TILT_Z 16384
#endif // ROLL_PLUS_VERTICAL

#if ( CONTROL_TYPE == ROLL_PLUS_TILT )
#define CONTROL_TEXT "roll and 12 deg tilt"
#define ROLL_ENABLE 1
#define YAW_PITCH_ENABLE 1
#define TILT_X 2409
#define TILT_Y 2409
#define TILT_Z 16026
#endif // ROLL_PLUS_TILT

#if ( CONTROL_TYPE == TILT_PATTERN )
#include "tilt_defs.h"
#define CONTROL_TEXT "tilt pattern defined in tilt_defs.h"
#define ROLL_ENABLE 1
#define YAW_PITCH_ENABLE 1
#define TILT_X 0
#define TILT_Y 0
#define TILT_Z 16384
#endif // 


#ifndef CONTROL_TYPE
#error "no control type defined"
#endif // CONTROL_TYPE

#define DATE "12/17/2023."
#define REVISION "VOS_port_RV3, 50 Hz.\r\nAlso supports gain scheduling"
#define MAX_TILT_ANGLE ( 7.5 ) // degrees
#define MAX_TILT_RATE ( 100.0 ) // degrees per second
#define MAX_TILT_PULSE_WIDTH ( 250.0 ) // microseconds
#define MAX_SPIN_RATE ( 1000.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 250.0 ) // microseconds
#define MAX_ROLL_ANGLE ( 360 ) // degrees

#define GYRO_RANGE ( 1000 )
//#define CALIBRATION ( 0.9945 )
#define CALIBRATION ( 0.9972 )
//#define GYRO_RANGE ( 500 )

#define CUSTOM_OFFSETS
// If the accelerometer offsets have been measured,
// enter their actual values in this section as integers,
// otherwise set them to zero.
#define XACCEL_OFFSET	( 0 )
#define YACCEL_OFFSET	( 0 )
#define ZACCEL_OFFSET	( 0 )

// Initial estimates of gyro offsets are set to zero.
// Actual gyro offsets are automatically computed while waiting for launch.
#define XRATE_OFFSET	( 0 )
#define YRATE_OFFSET	( 0 )
#define ZRATE_OFFSET	( 0 )

////////////////////////////////////////////////////////////////////////////////
// Set this value to your GPS type.  (Set to GPS_STD, GPS_UBX_2HZ, GPS_UBX_4HZ, GPS_MTEK, GPS_NMEA, or GPS_NONE)
#define GPS_TYPE                            GPS_STD

// Note: As of MatrixPilot 3.0, Dead Reckoning and Wind Estimation are automatically enabled.

// Define MAG_YAW_DRIFT to be 1 to use magnetometer for yaw drift correction.
// Otherwise, if set to 0 the GPS will be used.
// If you select this option, you also need to set magnetometer options in
// the magnetometerOptions.h file, including declination and magnetometer type.
#define MAG_YAW_DRIFT                       0

// Set this to 1 if you want the UAV Dev Board to fly your plane without a radio transmitter or
// receiver. (Totally autonomous.)  This is just meant for simulation and debugging.  It is not
// recommended that you actually use this option, since you'd have no manual control to fall
// back on if things go wrong.  It may not even be legal in your area.
#define NORADIO                             0


////////////////////////////////////////////////////////////////////////////////
// Configure Input and Output Channels
//
// NUM_INPUTS: Set to 0-5 
#define NUM_INPUTS                          0

// NUM_OUTPUTS: Set to 3, 4, 5, or 6
#define NUM_OUTPUTS                         8

// Channel numbers for each output
// Use as is, or edit to match your setup.
//   - Only assign each channel to one output purpose
//   - If you don't want to use an output channel, set it to CHANNEL_UNUSED
//


////////////////////////////////////////////////////////////////////////////////
// The Failsafe Channel is the RX channel that is monitored for loss of signal
// Make sure this is set to a channel you actually have plugged into the UAV Dev Board!
//
// For a receiver that remembers a failsafe value for when it loses the transmitter signal,
// like the Spektrum AR6100, you can program the receiver's failsafe value to a value below
// the normal low value for that channel.  Then set the FAILSAFE_INPUT_MIN value to a value
// between the receiver's programmed failsafe value and the transmitter's normal lowest
// value for that channel.  This way the firmware can detect the difference between a normal
// signal, and a lost transmitter.
//
// FAILSAFE_INPUT_MIN and _MAX define the range within which we consider the radio on.
// Normal signals should fall within about 2000 - 4000.
#define FAILSAFE_INPUT_CHANNEL              CHANNEL_UNUSED
#define FAILSAFE_INPUT_MIN                  1500
#define FAILSAFE_INPUT_MAX                  4500


////////////////////////////////////////////////////////////////////////////////
// Serial Output BAUD rate for status messages
//  19200, 38400, 57600, 115200, 230400, 460800, 921600 // yes, it really will work at this rate
#define SERIAL_BAUDRATE                     38400 // for 20 records per second
//#define SERIAL_BAUDRATE                     19200 // default
//#define SERIAL_BAUDRATE                     115200 // high speed for 40 records per second

////////////////////////////////////////////////////////////////////////////////
// Control gains.
// All gains should be positive real numbers.

// SERVOSAT limits servo throw by controlling pulse width saturation.
// set it to 1.0 if you want full servo throw, otherwise set it to the portion that you want
//#define SERVOSAT                            1.0
