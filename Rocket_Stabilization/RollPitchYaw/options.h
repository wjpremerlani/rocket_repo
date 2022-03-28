#define SERIAL_BAUDRATE                     19200

#define GROUND_TEST (1)
#define POLARITY -

#define DEAD_BAND 60
#define CENTER 2*1498
#define ROLL_RATE_ENABLE 1

#define PRELAUNCH_DELAY 120
#define FLIGHT_TIME 60

#define LOCKOUT_ROLL 180 // 180 degrees
#define LOCKOUT_COS_TILT 14189 // cosine 30 degrees

#define CONTROL_TEXT "roll only"
#define ROLL_ENABLE 1
#define YAW_PITCH_ENABLE 0
#define TILT_X 0
#define TILT_Y 0

#define DATE "3/28/2022"
#define REVISION "despin"

#define MAX_SPIN_RATE ( 1000.0 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 500.0 ) // microseconds
#define MAX_ROLL_ANGLE ( 360 ) // degrees

#define GYRO_RANGE ( 1000 )
//#define CALIBRATION ( 0.9945 )
#define CALIBRATION ( 0.9972 )
//#define GYRO_RANGE ( 500 )

// legacy defines, not used for despin
#define NUM_INPUTS                          0
#define NUM_OUTPUTS                         8

