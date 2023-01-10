// extra line for testing sourcetree
#define SERIAL_BAUDRATE                     19200

#define GROUND_TEST (0)
#define POLARITY -

#define DEAD_BAND 0
//#define CENTER 2*1507
#define CENTER ((uint16_t)32*1507)
#define ROLL_RATE_ENABLE 0

#define PRELAUNCH_DELAY 10
#define FLIGHT_TIME 60

#define LOCKOUT_ROLL 4*330 // 1320 degrees
//#define LOCKOUT_COS_TILT 14189 // cosine 30 degrees
#define LOCKOUT_COS_TILT -17000 // has the effect of ignoring tilt

#define CONTROL_TEXT "roll only"
#define ROLL_ENABLE 1
#define YAW_PITCH_ENABLE 0
#define TILT_X 0
#define TILT_Y 0

#define DATE "1/10/2023"
#define REVISION "despin"

#define MAX_SPIN_RATE ( 2000 ) // degrees per second
#define MAX_SPIN_PULSE_WIDTH ( 500.0 ) // microseconds
#define MAX_ROLL_ANGLE ( 4*360 ) // 1440 degrees

#define GYRO_RANGE ( 1000 )
#define CALIBRATION ( 0.9945 )


