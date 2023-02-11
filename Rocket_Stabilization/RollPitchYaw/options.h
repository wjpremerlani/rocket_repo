
#define CUSTOM_OFFSETS // this is the best option for rockets. you will need to define offsets

// define GROUND_TEST to be 1 for ground testing, set to 0 for flight
// for ground testing, launch is triggered by enabling the control by pulling SCL to ground
// this option also produces extra output such as gyro offsets
//#define GROUND_TEST ( 1 ) 
#define DEBUG_NO_MIX ( 0 )

// the following allows multiple sets of options to be saved in one file
#define FLORIN (0)
#define JIM (0)
#define BILL (0)
#define WAYNE_BRD4 (0)
#define WAYNE_BRD5 (0)
#define WAYNE_BRD6 (0)
#define WAYNE_BRD7 (0)
#define WAYNE_BRD8 (1)

// Florin's board
#if( FLORIN == 1)
// identify your board any way you want and set the date
#define BOARD "Florin #xx"
#define DATE "10/17/2020"
#if ( GROUND_TEST == 1)
#define REVISION "rev21, ground test"
#else
#define REVISION "rev21, flight ready"
#endif // GROUND_TEST
// tilt angle that produces maximum tilt response
#define MAX_TILT_ANGLE ( 7.5 ) // degrees
// servo PWM signal at max tilt
#define MAX_TILT_PULSE_WIDTH ( 250.0 ) // microseconds
// spin rate that produces maximum roll rate feedback
#define MAX_SPIN_RATE ( 500.0 ) // degrees per second
// servo PWM signal at max roll rate
#define MAX_SPIN_PULSE_WIDTH ( 250.0 ) // microseconds
// angle at which the heading hold integrator saturates
#define MAX_ROLL_ANGLE ( 180 ) // degrees

// these are parameters for intentional tilt control offset
// these values will turn it off all together
#define EARTH_TILT_X ( 0 )
#define EARTH_TILT_Y ( 0 )
#define TILT_DURATION ( 99 )

// this turns out to perform better than the 500 degree/sec range,
// so it is recommended
#define GYRO_RANGE ( 1000 )
// gyro calibration factor
#define CALIBRATION ( 1.000 )

// These are your sensor offsets.
// It is important to get the accelerometer offsets correct.
// The gyro values do not matter as much, they are starting points,
// but getting them approximately correct will reduce initialization time.
// Typical values are shown
#define XACCEL_OFFSET	( 100 )
#define YACCEL_OFFSET	( -60 )
#define ZACCEL_OFFSET	( -120 )
#define XRATE_OFFSET	( -30 )
#define YRATE_OFFSET	( 32 )
#define ZRATE_OFFSET	( 21 )
#endif // FLORIN



// Jim's first board, now its Tyler's
/*
#define BOARD "Tyler's Board"
#define DATE "12/15/2019"
#define REVISION "rev17a"
#define MAX_TILT_ANGLE ( 7.5 ) // degrees
#define MAX_TILT_PULSE_WIDTH ( 250.0 ) // microseconds
#define MAX_SPIN_RATE ( 750.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 250.0 ) // microseconds

#define GYRO_RANGE ( 1000 )

#define XACCEL_OFFSET	( 244 )
#define YACCEL_OFFSET	( -27 )
#define ZACCEL_OFFSET	( -960 )
#define XRATE_OFFSET	( -43 )
#define YRATE_OFFSET	( -47 )
#define ZRATE_OFFSET	( 6 )

*/


// Jim's second board
#if ( JIM == 1 )
#define USE_TILT (1)
#define GROUND_TEST (1)
#define BOARD "Jim Brd2"
#define DATE "7/26/2021"
#if ( GROUND_TEST == 1)
#define REVISION "rev23a, snake, ground test"
#else
#define REVISION "rev23a, snake, flight ready"
#endif // GROUND_TEST
#define MAX_TILT_ANGLE ( 7.5 ) // degrees
#define MAX_TILT_RATE ( 100.0 ) // degrees per second
#define MAX_TILT_PULSE_WIDTH ( 250.0 ) // microseconds
#define MAX_SPIN_RATE ( 1000.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 250.0 ) // microseconds
#define MAX_ROLL_ANGLE ( 360 ) // degrees

#include "tilt_defs.h"

#define GYRO_RANGE ( 1000 )
//#define CALIBRATION ( 0.9945 )
#define CALIBRATION ( 0.9972 )
//#define GYRO_RANGE ( 500 )

#define XACCEL_OFFSET	( 367 )
#define YACCEL_OFFSET	( -66 )
#define ZACCEL_OFFSET	( -128 )
#define XRATE_OFFSET	( -115 )
#define YRATE_OFFSET	( 32 )
#define ZRATE_OFFSET	( 21 )
#endif // JIM


/*
// Ray's first board
#define BOARD "Ray Brd1"
#define DATE "11/27/2016"
#define REVISION "rev17a"
#define MAX_TILT_ANGLE ( 7.5 ) // degrees
#define MAX_TILT_PULSE_WIDTH ( 250.0 ) // microseconds
#define MAX_SPIN_RATE ( 750.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 250.0 ) // microseconds

#define GYRO_RANGE ( 1000 )

#define XACCEL_OFFSET	( 291 )
#define YACCEL_OFFSET	( -85 )
#define ZACCEL_OFFSET	( 531 )
#define XRATE_OFFSET	( -44 )
#define YRATE_OFFSET	( 29 )
#define ZRATE_OFFSET	( 1 )
*/

/*
// Bill's board 1
#define GYRO_RANGE 1000
#define BOARD "BillsBrd1"
#define DATE "5/22/2016"
#define REVISION "Rev15b"
//#define MOUNT_ORIENTATION VERTICAL_MOUNT
//#define MAX_TILT ( 0.25 )
//#define MAX_SPIN_RATE ( 2160.0 ) // after third flight
//#define MAX_SPIN_RATE ( 500.0 )

#define MAX_TILT_ANGLE ( 7.5 ) // degrees
#define MAX_TILT_PULSE_WIDTH ( 250.0 ) // microseconds

//#define MAX_SPIN_RATE ( 250.0 ) // degrees per second
//#define MAX_SPIN_PULSE_WIDTH ( 250.0 ) // microseconds

#define MAX_SPIN_RATE ( 90.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 250.0 ) // microseconds

#define XACCEL_OFFSET	( 480 )
#define YACCEL_OFFSET	( -53 )
#define ZACCEL_OFFSET	( -1690 )
//#define XRATE_OFFSET	( -24 )
//#define YRATE_OFFSET	( 143 )
//#define ZRATE_OFFSET	( -33 )

#define XRATE_OFFSET	( 0 )
#define YRATE_OFFSET	( 0 )
#define ZRATE_OFFSET	( 0 )
*/
// Bill's board 2
#if ( BILL == 1)
#undef GROUND_TEST
#define GROUND_TEST ( 1 )
#define BOARD "BillsBrd2"
#define DATE "10/18/2020"
#define REVISION "Rev22 ground test"
#define GYRO_RANGE ( 1000 )
#define CALIBRATION ( 0.9924 )

//#define EARTH_TILT_X ( -2011 )
//#define EARTH_TILT_Y ( 2011 )

#define EARTH_TILT_X ( 0 )
#define EARTH_TILT_Y ( 0 ) 
#define TILT_DURATION ( 12 ) //seconds

#define MAX_TILT_ANGLE ( 7.5 ) // degrees
#define MAX_TILT_PULSE_WIDTH ( 250.0 ) // microseconds

//#define MAX_SPIN_RATE ( 250.0 ) // degrees per second
//#define MAX_SPIN_PULSE_WIDTH ( 250.0 ) // microseconds
//#define MAX_ROLL_ANGLE ( 720 ) // degrees

//#define NO_SPIN_CONTROL
#define MAX_SPIN_RATE ( 100.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 250.0 ) // microseconds
#define MAX_ROLL_ANGLE ( 90 ) // degrees

#define XACCEL_OFFSET	( 312 )
#define YACCEL_OFFSET	( -98 )
#define ZACCEL_OFFSET	( -443 )
//#define XRATE_OFFSET	( -22 )
//#define YRATE_OFFSET	( 17 )
//#define ZRATE_OFFSET	( -29 )
#define XRATE_OFFSET	( -88 )
#define YRATE_OFFSET	( -7 )
#define ZRATE_OFFSET	( -21 )
#endif
// Ray's first board
/*
#define XACCEL_OFFSET	( 311 )
#define YACCEL_OFFSET	( 12 )
#define ZACCEL_OFFSET	( 44 )
#define XRATE_OFFSET	( -132 )
#define YRATE_OFFSET	( 59 )
#define ZRATE_OFFSET	( -18 )

#define BOARD "Ray Brd1"
#define DATE "5/25/2015"
#define REVISION "Rev14"
//#define MOUNT_ORIENTATION HORIZONTAL_MOUNT
#define MAX_TILT ( 1.0 )
#define MAX_SPIN_RATE ( 1500.0 ) // after third flight
*/

// SN1
/*
#define XACCEL_OFFSET	( 309 )
#define YACCEL_OFFSET	( -9 )
#define ZACCEL_OFFSET	( -609 )
#define XRATE_OFFSET	( 79 )
#define YRATE_OFFSET	( -17 )
#define ZRATE_OFFSET	( -13 )

#define BOARD "board SN1"
#define DATE "7/19/2015"
#define REVISION "Rev14"
#define MAX_TILT ( 1.0 )
#define MAX_SPIN_RATE ( 1500.0 )
*/

// SN2
/*
#define XACCEL_OFFSET	( 232 )
#define YACCEL_OFFSET	( 13 )
#define ZACCEL_OFFSET	( -129 )
#define XRATE_OFFSET	( 77 )
#define YRATE_OFFSET	( 44 )
#define ZRATE_OFFSET	( 11 )

#define BOARD "board SN2"
#define DATE "7/19/2015"
#define REVISION "Rev14"
#define MAX_TILT ( 1.0 )
#define MAX_SPIN_RATE ( 1500.0 )
*/

// SN3
/*
#define XACCEL_OFFSET	( 328 )
#define YACCEL_OFFSET	( -134 )
#define ZACCEL_OFFSET	( -706 )
#define XRATE_OFFSET	( -94 )
#define YRATE_OFFSET	( 43 )
#define ZRATE_OFFSET	( -21 )

#define BOARD "board SN3"
#define DATE "7/19/2015"
#define REVISION "Rev14"
#define MAX_TILT ( 1.0 )
#define MAX_SPIN_RATE ( 1500.0 )
*/


// SN4
#if ( WAYNE_BRD4 == 1 )
#error "board 4 was lost"
#define USE_TILT (0)
#define MOUNT_ORIENTATION VERTICAL_MOUNT
//#define MOUNT_ORIENTATION HORIZONTAL_MOUNT
#define DETECT_APOGEE
#define NO_MIXING

#define XACCEL_OFFSET	( 164 )
#define YACCEL_OFFSET	( -39 )
#define ZACCEL_OFFSET	( 92 )
#define XRATE_OFFSET	( -85 )
#define YRATE_OFFSET	( -37 )
#define ZRATE_OFFSET	( -11 )
#define CALIBRATION ( 1.0000 )

#define BOARD "board SN4, Wayne"
#if (( GROUND_TEST == 1 ))
#define DATE "Dec. 14, 2020, ground test"
#else
#define DATE "Dec. 14, 2020, flight ready"
#endif // GROUND_TEST 
#define REVISION "Rev22, vertical, apogee detect"
#define MAX_TILT_ANGLE ( 30.0 ) // degrees
#define MAX_TILT_PULSE_WIDTH ( 500.0 ) // microseconds
#define MAX_SPIN_RATE ( 300.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 500.0 ) // microseconds
#define MAX_ROLL_ANGLE ( 180 )

#define GYRO_RANGE ( 1000 )
#endif // WAYNE_BRD4

// SN5
#if ( WAYNE_BRD5 == 1 )
#define GROUND_TEST (0)
#define USE_TILT (0)
#define MOUNT_ORIENTATION VERTICAL_MOUNT
//#define MOUNT_ORIENTATION HORIZONTAL_MOUNT
#define DETECT_APOGEE
#define NO_MIXING

#define PWM1_CENTER (3020)
#define PWM2_CENTER (2914)
#define PWM3_CENTER (3112)

#define PWM1_SIGN +
#define PWM2_SIGN -
#define PWM3_SIGN -

#define XACCEL_OFFSET	( 330 )
#define YACCEL_OFFSET	( -75 )
#define ZACCEL_OFFSET	( -405 )
#define XRATE_OFFSET	( -44 )
#define YRATE_OFFSET	( 10 )
#define ZRATE_OFFSET	( 15 )
#define CALIBRATION ( 1.0032 )

#define BOARD "SN5, Wayne"
#if (( GROUND_TEST == 1 ))
#define DATE "2/9/2023, gnd test"
#else
#define DATE "2/9/2023, flt rdy"
#endif // GROUND_TEST 
#define REVISION "R23, vert, apogee det"
#define MAX_TILT_ANGLE ( 30.0 ) // degrees
#define MAX_TILT_RATE ( 300.0 ) // degrees per second
#define MAX_TILT_PULSE_WIDTH ( 500.0 ) // microseconds
#define MAX_SPIN_RATE ( 900.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 300.0 ) // microseconds
#define MAX_ROLL_ANGLE ( 540 )

#define GYRO_RANGE ( 1000 )
#endif // WAYNE_BRD5
// SN6
#if ( WAYNE_BRD6 == 1 )
#define GROUND_TEST (1)
#define USE_TILT (0)
#define MOUNT_ORIENTATION VERTICAL_MOUNT
//#define MOUNT_ORIENTATION HORIZONTAL_MOUNT
#define DETECT_APOGEE
#define NO_MIXING

#define PWM1_CENTER (3020)
#define PWM2_CENTER (2944)
#define PWM3_CENTER (3154)

#define PWM1_SIGN +
#define PWM2_SIGN -
#define PWM3_SIGN -

#define XACCEL_OFFSET	( 285 )
#define YACCEL_OFFSET	( 10 )
#define ZACCEL_OFFSET	( -1197 )
#define XRATE_OFFSET	( 40 )
#define YRATE_OFFSET	( 90 )
#define ZRATE_OFFSET	( 25 )
#define CALIBRATION ( 0.9961 )

#define BOARD "SN6, Wayne"
#if (( GROUND_TEST == 1 ))
#define DATE "2/11/2022, gnd test"
#else
#define DATE "2/11/2022, flt rdy"
#endif // GROUND_TEST 
#define REVISION "R23, vert, apogee det"
#define MAX_TILT_ANGLE ( 30.0 ) // degrees
#define MAX_TILT_RATE ( 300.0 ) // degrees per second
#define MAX_TILT_PULSE_WIDTH ( 500.0 ) // microseconds
#define MAX_SPIN_RATE ( 300.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 300.0 ) // microseconds
#define MAX_ROLL_ANGLE ( 180 )

#define GYRO_RANGE ( 1000 )
#endif // WAYNE_BRD6

// SN7
#if ( WAYNE_BRD7 == 1 )
#define GROUND_TEST (0)
#define USE_TILT (0)
#define MOUNT_ORIENTATION VERTICAL_MOUNT
//#define MOUNT_ORIENTATION HORIZONTAL_MOUNT
#define DETECT_APOGEE
#define NO_MIXING

#define PWM1_CENTER (3020)
#define PWM2_CENTER (2914)
#define PWM3_CENTER (3112)

#define PWM1_SIGN +
#define PWM2_SIGN -
#define PWM3_SIGN -

#define XACCEL_OFFSET	( 156 )
#define YACCEL_OFFSET	( -17 )
#define ZACCEL_OFFSET	( -383 )
#define XRATE_OFFSET	( 15 )
#define YRATE_OFFSET	( -18 )
#define ZRATE_OFFSET	( -2 )
#define CALIBRATION ( 1.0032 )

#define BOARD "SN7, Wayne"
#if (( GROUND_TEST == 1 ))
#define DATE "2/11/2023, gnd test"
#else
#define DATE "2/11/2023, flt rdy"
#endif // GROUND_TEST 
#define REVISION "R23, vert, apogee det"
#define MAX_TILT_ANGLE ( 30.0 ) // degrees
#define MAX_TILT_RATE ( 300.0 ) // degrees per second
#define MAX_TILT_PULSE_WIDTH ( 500.0 ) // microseconds
#define MAX_SPIN_RATE ( 900.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 300.0 ) // microseconds
#define MAX_ROLL_ANGLE ( 540 )

#define GYRO_RANGE ( 1000 )
#endif // WAYNE_BRD7

// SN7
#if ( WAYNE_BRD8 == 1 )
#define GROUND_TEST (1)
#define USE_TILT (0)
#define MOUNT_ORIENTATION VERTICAL_MOUNT
//#define MOUNT_ORIENTATION HORIZONTAL_MOUNT
#define DETECT_APOGEE
#define NO_MIXING

#define PWM1_CENTER (3020)
#define PWM2_CENTER (2914)
#define PWM3_CENTER (3112)

#define PWM1_SIGN +
#define PWM2_SIGN -
#define PWM3_SIGN -

#define XACCEL_OFFSET	( 185 )
#define YACCEL_OFFSET	( -86 )
#define ZACCEL_OFFSET	( -340 )
#define XRATE_OFFSET	( -39 )
#define YRATE_OFFSET	( -43 )
#define ZRATE_OFFSET	( -50 )
#define CALIBRATION ( 1.0032 )

#define BOARD "SN8, Wayne"
#if (( GROUND_TEST == 0 ))
#define DATE "2/11/2023, gnd test"
#else
#define DATE "2/11/2023, flt rdy"
#endif // GROUND_TEST 
#define REVISION "R23, vert, apogee det"
#define MAX_TILT_ANGLE ( 30.0 ) // degrees
#define MAX_TILT_RATE ( 300.0 ) // degrees per second
#define MAX_TILT_PULSE_WIDTH ( 500.0 ) // microseconds
#define MAX_SPIN_RATE ( 900.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 300.0 ) // microseconds
#define MAX_ROLL_ANGLE ( 540 )

#define GYRO_RANGE ( 1000 )
#endif // WAYNE_BRD7

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
#define SERIAL_BAUDRATE                     19200 // default
//#define SERIAL_BAUDRATE                     115200 // high speed for 40 records per second

////////////////////////////////////////////////////////////////////////////////
// Control gains.
// All gains should be positive real numbers.

// SERVOSAT limits servo throw by controlling pulse width saturation.
// set it to 1.0 if you want full servo throw, otherwise set it to the portion that you want
//#define SERVOSAT                            1.0
