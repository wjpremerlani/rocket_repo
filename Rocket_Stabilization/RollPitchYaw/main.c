// This file is part of the MatrixPilot RollPitchYaw demo.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2013 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


// main program for testing the IMU.


#include "../libDCM/libDCM.h"
#include "../libUDB/heartbeat.h"

// Used for serial debug output
#include <stdio.h>

#include <stdint.h>

//#define DEBUG_GAIN_SCHEDULE
#ifdef DEBUG_GAIN_SCHEDULE
#define MAX_ROLL_BINARY_ MAX_ROLL_BINARY
#define TILT_ALLOTMENT_INT_ TILT_ALLOTMENT_INT
#define SPIN_ALLOTMENT_INT_ SPIN_ALLOTMENT_INT
#define TOTAL_DEFLECTION_ TOTAL_DEFLECTION
#define MAX_ROLL_ANGLE_ MAX_ROLL_ANGLE
#define TILT_GAIN_ TILT_GAIN
#define SPIN_GAIN_ SPIN_GAIN
#define TILT_RATE_GAIN_ TILT_RATE_GAIN
#else
#define MAX_ROLL_BINARY_ max_roll_binary
#define TILT_ALLOTMENT_INT_ tilt_allotment_int
#define SPIN_ALLOTMENT_INT_ spin_allotment_int
#define TOTAL_DEFLECTION_ total_deflection
#define MAX_ROLL_ANGLE_ max_roll_angle
#define TILT_GAIN_ tilt_gain
#define SPIN_GAIN_ spin_gain
#define TILT_RATE_GAIN_ tilt_rate_gain
#endif // DEBUG_GAIN_SCHEDULE

char debug_buffer[512];
int db_index = 0;
void send_debug_line(void);

extern int16_t omega[] ;
extern int16_t omegacorrI[] ;
extern fractional theta[3] ;
int16_t delta_roll ;
union longww roll_angle_32 = { 0 } ;
int16_t roll_deviation ;

struct relative2D roll_reference ;
int16_t roll_angle ;
int16_t rect_to_polar16(struct relative2D *xy);
int16_t signed_saturate(float x );
uint16_t unsigned_saturate(float x) ;

#define MAX_ROLL_BINARY ((int32_t) ((int32_t)286*(int32_t)MAX_ROLL_ANGLE ))
int32_t max_roll_binary = MAX_ROLL_BINARY ;

int16_t apogee = 0 ;
int16_t tilted = 0 ;

int16_t tilt_count = 0 ;

int16_t tilt_pwm ;
float max_tilt  ;
int16_t max_tilt_rate ;
int16_t roll_pwm ;
int16_t max_roll ;
int16_t max_roll_rate ;
uint16_t gain_time ;
int16_t first_gain_retrieved = 0 ;
int16_t roll_step = 0 ;

#if (CONTROL_TYPE == TILT_PATTERN)
struct tilt_def {
	int16_t tilt ;
	int16_t b ;
	int16_t x ;
	int16_t y ;
    int16_t roll_step ;
	uint16_t t	;		
};
struct tilt_def tilt_defs[] = TILT_DEFS ;
uint16_t NUM_TILTS = sizeof(tilt_defs)/sizeof(tilt_defs[0]) ;
#endif // TILT_PATTERN

#ifdef GAIN_SCHEDULING
#include "gain_defs.h"
struct gain_def {   
    int16_t tilt_pwm ;
    float max_tilt ;
    int16_t max_tilt_rate ;
    int16_t roll_pwm ;
    int16_t max_roll ;
    int16_t max_roll_rate ;
    uint16_t time ;
};
struct gain_def gain_defs[] = GAIN_DEFS ;
uint16_t NUM_GAINS = sizeof(gain_defs)/sizeof(gain_defs[0]) ;
#endif // GAIN_SCHEDULING

void compute_control_gains(void) ;

#define RECORD_OFFSETS	( 0 ) // set to 1 in order to record accelerometer and gyro offsets in telemetry

int main(void)
{
	_TRISA2 = 1 ; // SCL is input pin for enabling yaw/pitch control
	_TRISA3 = 1 ; // SDA is input pin for enabling roll control
	mcu_init();

	// Set up the libraries
	udb_init();
	dcm_init();

	udb_serial_set_rate(SERIAL_BAUDRATE);

	LED_GREEN = LED_OFF;
	
	roll_angle_32.WW = 0 ;

	// Start it up!
	while (1)
	{
		udb_run();
	}

	return 0;
}

// Called high priority
void udb_heartbeat_high_callback(void)
{
	static int count = 0;

	if (!dcm_flags._.calib_finished)
	{
		if (++count > 20)
		{
			count = 0;
		}
	}
}

// Called every time we get gps data (1, 2, or 4 Hz, depending on GPS config)
void dcm_callback_gps_location_updated(void)
{
	// Blink GREEN led to show that the GPS is communicating
	udb_led_toggle(LED_GREEN);
}

int16_t hundredths = 0 ;
int16_t tenths = 0 ;
int16_t seconds = 0 ;
int16_t minutes = 0 ;

#define TILT_ALLOTMENT ( 2.0*MAX_TILT_PULSE_WIDTH )
#define SPIN_ALLOTMENT ( 2.0*MAX_SPIN_PULSE_WIDTH )
#define TILT_ALLOTMENT_INT (( uint16_t ) TILT_ALLOTMENT)
#define SPIN_ALLOTMENT_INT (( uint16_t ) SPIN_ALLOTMENT)
#define TOTAL_DEFLECTION ( TILT_ALLOTMENT_INT + SPIN_ALLOTMENT_INT )

int16_t tilt_allotment_int = (( uint16_t ) TILT_ALLOTMENT) ;
int16_t spin_allotment_int = (( uint16_t ) SPIN_ALLOTMENT) ;
int16_t total_deflection = ( TILT_ALLOTMENT_INT + SPIN_ALLOTMENT_INT );
int16_t max_roll_angle = MAX_ROLL_ANGLE ;

#define TILT_GAIN ( 4.0 * TILT_ALLOTMENT / ( MAX_TILT_ANGLE / 57.3 ) ) 
#define SPIN_GAIN ( SPIN_ALLOTMENT * ( 256.0 / 65.0) * ( 256.0 / MAX_SPIN_RATE ) )
#define TILT_RATE_GAIN ( TILT_ALLOTMENT * ( 256.0 / 65.0) * ( 256.0 / MAX_TILT_RATE ) )

float tilt_gain = TILT_GAIN ;
float spin_gain = SPIN_GAIN ;
float tilt_rate_gain = TILT_RATE_GAIN ;

int16_t target_earth_frame_tilt[3];
int16_t rmat_column_1[3];
int16_t rmat_column_2[3];
int16_t column_1_dot_target ;
int16_t column_2_dot_target ;

uint16_t tilt_index = 0 ;
uint16_t tilt_print_index = 0 ;
uint16_t gain_index = 0 ;
uint16_t gain_print_index = 0 ;
uint16_t tilt_t ;

int16_t controlModeYawPitch = YAW_PITCH_ENABLE ;
int16_t controlModeRoll = ROLL_ENABLE ;
int16_t launched = 0 ;
int16_t roll_saturated = 0 ;
int16_t accelEarthVertical = 0 ;
int32_t velocityEarthVertical = 0 ;
int16_t launch_count = 0 ;
int16_t roll_saturated_count = 0 ;


int16_t roll_feedback_horizontal_pitch = 0 ;
int16_t roll_feedback_horizontal_yaw = 0 ;
int16_t total_roll_feedback_horizontal = 0 ;

int16_t pitch_feedback_horizontal = 0 ;
int16_t yaw_feedback_horizontal = 0 ;

int16_t saturate( int16_t max , int16_t input )
{
	int16_t abs_max ;
	abs_max = abs( max) ; // just to make sure
	if ( input > abs_max )
	{
		return abs_max ;
	}
	else if ( input < -abs_max)
	{
		return -abs_max ;
	}
	else
	{
		return input ;
	}
}

int32_t saturate_32( int32_t max , int32_t input )
{
	int32_t abs_max ;
	abs_max = abs( max) ; // just to make sure
	if ( input > abs_max )
	{
		return abs_max ;
	}
	else if ( input < -abs_max)
	{
		return -abs_max ;
	}
	else
	{
		return input ;
	}
}

int16_t remove_offset ( int16_t value , int16_t offset )
{
	return (int16_t) saturate_32 ( ( int32_t ) 0x7FFF , (int32_t) value - (int32_t) offset ) ;
}

int16_t tilt_feedback ( int16_t tilt , int16_t tilt_rate )
{
	union longww accum ;
	accum.WW = __builtin_mulsu ( tilt , unsigned_saturate( TILT_GAIN_) ) ;
#if ( GYRO_RANGE == 500 )
	accum.WW += __builtin_mulsu (  tilt_rate , unsigned_saturate( 2.0*TILT_RATE_GAIN_ ) ) ; // 2 is because use of drift corrected values instead of raw values
#elif ( GYRO_RANGE == 1000 )
	accum.WW += __builtin_mulsu (  tilt_rate , unsigned_saturate ( 4.0*TILT_RATE_GAIN_ ) ) ; // 1000 degree per second
#else
#error set GYRO_RANGE to 500 or 1000 in options.h
#endif // GYRO_RANGE
	return saturate ( TILT_ALLOTMENT_INT_ , accum._.W1 )  ;
}

//#define USE_ROLL_MARGIN  // uncomment this line if you want to allocate unused tilt deflection to roll control
void roll_feedback ( int16_t pitch_feedback , int16_t yaw_feedback ,  int16_t roll_rate , int16_t roll_deviation ,
							int16_t * roll_feedback_pitch_pntr , int16_t * roll_feedback_yaw_pntr , int16_t * total_roll_feedback_pntr )
{
	union longww accum ;
	int16_t roll_margin_yaw ;
	int16_t roll_margin_pitch ;
	int16_t yaw_margin_minus_pitch_margin_over_2 ;
	int16_t abs_yaw_margin_minus_pitch_margin_over_2 ;
	int16_t net_roll_margin ;
	int16_t net_roll_deflection ;
	int16_t abs_net_roll_deflection ;
	int16_t roll_pitch ;
	int16_t roll_yaw ;

#ifdef USE_ROLL_MARGIN 
	roll_margin_pitch = TOTAL_DEFLECTION_ - abs ( pitch_feedback ) ;
	roll_margin_yaw = TOTAL_DEFLECTION_ - abs ( yaw_feedback ) ;
#else
    roll_margin_pitch = TILT_ALLOTMENT_INT_ ;
    roll_margin_yaw = TILT_ALLOTMENT_INT_ ;
#endif // USE_ROLL_MARGIN
	yaw_margin_minus_pitch_margin_over_2 = ( roll_margin_yaw - roll_margin_pitch ) / 2 ;
	abs_yaw_margin_minus_pitch_margin_over_2 = abs ( yaw_margin_minus_pitch_margin_over_2 ) ;
	
#ifdef NO_MIXING
	net_roll_margin = SPIN_ALLOTMENT_INT_ ;
#else	
	net_roll_margin = ( roll_margin_pitch + roll_margin_yaw ) / 2 ;
#endif

#if ( GYRO_RANGE == 500 )
	accum.WW = __builtin_mulsu (  roll_rate , unsigned_saturate  (2.0*SPIN_GAIN_) ) ; // 2 is because use of drift corrected values instead of raw values
#elif ( GYRO_RANGE == 1000 )
	accum.WW = __builtin_mulsu (  roll_rate , unsigned_saturate (4.0*SPIN_GAIN_) ) ; // 1000 degree per second
#else
#error set GYRO_RANGE to 500 or 1000 in options.h
#endif // GYRO_RANGE
	
	accum._.W1 += __builtin_divsd( __builtin_mulsu( roll_deviation , SPIN_ALLOTMENT_INT_ ) , MAX_ROLL_ANGLE_ ) ;
			
	net_roll_deflection = saturate ( net_roll_margin , accum._.W1 ) ;
	abs_net_roll_deflection = abs ( net_roll_deflection ) ;

	if ( abs_net_roll_deflection < abs_yaw_margin_minus_pitch_margin_over_2 ) // case 1
	{
		if ( roll_margin_yaw > roll_margin_pitch ) // case 1a
		{
			roll_yaw = 2*net_roll_deflection ;
			roll_pitch = 0 ;
		}
		else // case 1b
		{
			roll_yaw = 0 ;
			roll_pitch = 2*net_roll_deflection ;
		}
	}
	else // case 2
	{
		if ( net_roll_deflection > 0 ) // case 2a
		{
			roll_yaw = net_roll_deflection + yaw_margin_minus_pitch_margin_over_2 ;
			roll_pitch = net_roll_deflection - yaw_margin_minus_pitch_margin_over_2 ;
		}
		else // case 2b
		{
			roll_yaw = net_roll_deflection - yaw_margin_minus_pitch_margin_over_2 ;
			roll_pitch = net_roll_deflection + yaw_margin_minus_pitch_margin_over_2 ;
		}
	}
	* roll_feedback_pitch_pntr = roll_pitch ;
	* roll_feedback_yaw_pntr = roll_yaw ;
	* total_roll_feedback_pntr = net_roll_deflection ;
	return ;
}

#define VERTICAL_MOUNT  1 
#define HORIZONTAL_MOUNT  2 

// Called at HEARTBEAT_HZ, before sending servo pulses
void dcm_heartbeat_callback(void) // was called dcm_servo_callback_prepare_outputs()
{
	union longww accum;
	if (!dcm_flags._.calib_finished)
	{
		LED_GREEN = LED_OFF ;
	}
	else
	{
		LED_GREEN = LED_ON ;
		if ( launched == 1 )
		{
            tilt_count ++ ;
			LED_RED = LED_ON ;
			// update roll_angle_32
			// compute earth frame Z axis change in angle
			accum.WW = 0 ;
			accum.WW += ( __builtin_mulss( rmat[6] , theta[0] ) << 2 ) ;
			accum.WW += ( __builtin_mulss( rmat[7] , theta[1] ) << 2 ) ;
			accum.WW += ( __builtin_mulss( rmat[8] , theta[2] ) << 2 ) ;
#if ( GYRO_RANGE == 500)
			roll_angle_32.WW += accum._.W1 ;
#elif ( GYRO_RANGE == 1000)
			roll_angle_32.WW += accum._.W1 ;
			roll_angle_32.WW += accum._.W1 ;
#endif // GYRO_RANGE
			if ( roll_angle_32.WW > MAX_ROLL_BINARY_ )
			{
				roll_angle_32.WW = MAX_ROLL_BINARY_ ;
			}
			else if ( roll_angle_32.WW < - MAX_ROLL_BINARY_  )
			{
				roll_angle_32.WW = - MAX_ROLL_BINARY_ ;
			}
			roll_deviation = (int16_t)(roll_angle_32.WW/(int32_t)286);	
		}
		else
		{
			LED_RED = LED_OFF ;
		}
#ifdef GAIN_SCHEDULING
        {
            if ( launched == 1)
            {
                if (first_gain_retrieved == 0 )
                {
                    gain_index = 0 ;
                    tilt_pwm = gain_defs[gain_index].tilt_pwm ;
                    max_tilt = gain_defs[gain_index].max_tilt ;
                    max_tilt_rate = gain_defs[gain_index].max_tilt_rate ;
                    roll_pwm = gain_defs[gain_index].roll_pwm ;
                    max_roll = gain_defs[gain_index].max_roll ;
                    max_roll_rate = gain_defs[gain_index].max_roll_rate ;
                    compute_control_gains();
                    first_gain_retrieved = 1 ;
                }
                uint16_t gain_time = gain_defs[gain_index].time ;
                if ((tilt_count > 5*gain_time) &&(gain_index< NUM_GAINS-1))
                {   
                    gain_index++;
                    tilt_pwm = gain_defs[gain_index].tilt_pwm ;
                    max_tilt = gain_defs[gain_index].max_tilt ;
                    max_tilt_rate = gain_defs[gain_index].max_tilt_rate ;
                    roll_pwm = gain_defs[gain_index].roll_pwm ;
                    max_roll = gain_defs[gain_index].max_roll ;
                    max_roll_rate = gain_defs[gain_index].max_roll_rate ;
                    compute_control_gains();
                }
            }
        }
#endif // 
		{
#if ( CONTROL_TYPE == TILT_PATTERN )
            if ( launched == 1 )
            {
                tilt_t = tilt_defs[tilt_index].t ;
            	if ((tilt_count > 5*tilt_t) &&(tilt_index< NUM_TILTS-1))
                {   
                    roll_step = tilt_defs[tilt_index].roll_step ;
                    roll_angle_32.WW = roll_angle_32.WW - ((int32_t)roll_step)*((int32_t)286) ;
                    if ( roll_angle_32.WW > MAX_ROLL_BINARY_ )
                    {
                        roll_angle_32.WW = MAX_ROLL_BINARY_ ;
                    }
                    else if ( roll_angle_32.WW < - MAX_ROLL_BINARY_  )
                    {
                        roll_angle_32.WW = - MAX_ROLL_BINARY_ ;
                    }
                    roll_deviation = (int16_t)(roll_angle_32.WW/(int32_t)286);	
	
                    tilt_index++;
                }
                target_earth_frame_tilt[0] = - tilt_defs[tilt_index].x ;
                target_earth_frame_tilt[1] = - tilt_defs[tilt_index].y ;
                target_earth_frame_tilt[2] =  TILT_Z ;
            }
            else
            {
                target_earth_frame_tilt[0] = - TILT_X ;
                target_earth_frame_tilt[1] = - TILT_Y ;
                target_earth_frame_tilt[2] =  TILT_Z ;
            }
        }
#else
            target_earth_frame_tilt[0] = - TILT_X ;
            target_earth_frame_tilt[1] = - TILT_Y ;
            target_earth_frame_tilt[2] =  TILT_Z ;
        }
#endif // TILT_PATTERN
		{
			if ( ( _RA2 == 0 ) || ( _RA3 == 0 ) ) // ground test simulate launch 
			{
				launched = 1 ;
			}
		}
	

		if ( ( controlModeYawPitch == 1 )  )
		{
            rmat_column_1[0] = rmat[0];
            rmat_column_1[1] = rmat[3];
            rmat_column_1[2] = rmat[6];
            rmat_column_2[0] = rmat[1];
            rmat_column_2[1] = rmat[4];
            rmat_column_2[2] = rmat[7];
            column_1_dot_target = 2*VectorDotProduct(3,rmat_column_1,target_earth_frame_tilt);
            column_2_dot_target = 2*VectorDotProduct(3,rmat_column_2,target_earth_frame_tilt); 
            pitch_feedback_horizontal = tilt_feedback ( column_1_dot_target , -omega[1] ) ;
			yaw_feedback_horizontal = tilt_feedback ( column_2_dot_target , omega[0] ) ;
		}
		else
		{
			pitch_feedback_horizontal = 0 ;
			yaw_feedback_horizontal = 0 ;	
		}

		if ( ( controlModeRoll == 1 ) )
		{
			roll_feedback ( pitch_feedback_horizontal , yaw_feedback_horizontal , - omega[2]  , - roll_deviation ,
							&roll_feedback_horizontal_pitch , &roll_feedback_horizontal_yaw , &total_roll_feedback_horizontal ) ;
		}
		else
		{
			roll_feedback_horizontal_pitch = 0 ;
			roll_feedback_horizontal_yaw = 0 ;
			total_roll_feedback_horizontal = 0 ;
		}
		
		udb_pwOut[5] = 3000 ;
		udb_pwOut[6] = 3000 ;
		udb_pwOut[7] = 3000 ;
		udb_pwOut[8] = 3000 ;
        
 //       if (roll_saturated == 0) 
        if(1)
        {
            udb_pwOut[1] = roll_feedback_horizontal_pitch + pitch_feedback_horizontal + 3000 ;
            udb_pwOut[2] = roll_feedback_horizontal_yaw + yaw_feedback_horizontal + 3000 ;
            udb_pwOut[3] = roll_feedback_horizontal_pitch - pitch_feedback_horizontal + 3000 ;
            udb_pwOut[4] = roll_feedback_horizontal_yaw - yaw_feedback_horizontal + 3000 ;
        }
        else
        {
            udb_pwOut[1] = 3000 ;
            udb_pwOut[2] = 3000 ;
            udb_pwOut[3] = 3000 ;
            udb_pwOut[4] = 3000 ;           
        }

	}


#if ( OUTPUT_HZ == 10 )
	if (udb_heartbeat_counter % 5 == 0)
#elif (OUTPUT_HZ == 25)
		if (udb_heartbeat_counter % 2 == 0)
#else
#error "OUTPUT_HZ must be either 10 or 25"
#endif // OUTPUT_HZ 
	{
		if (dcm_flags._.calib_finished)
		{
			send_debug_line();
		}
	}
}

int16_t accelOn ;

#if ( GYRO_RANGE == 500 )
#define GYRO_FACTOR ( 65 ) // UDB5 sensitivity
#elif ( GYRO_RANGE == 1000 )
#define GYRO_FACTOR ( 32 ) // UDB5 sensitivity
#else
#error GYRO_RANGE not specified as 500 or 1000
#endif // GYRO_RANGE

uint16_t t_start = 0 ;
uint16_t t_end = 0 ;
uint16_t t_gain_start = 0 ;
uint16_t t_gain_end = 0 ;

int16_t line_number = 1 ;
extern union longww omegagyro_filtered[];
// Prepare a line of serial output and start it sending
void send_debug_line(void)
{
	db_index = 0;
	if( RECORD_OFFSETS == 1 )
	{
		sprintf( debug_buffer , "%i, %i, %i, %i, %i, %i\r\n" , 
			udb_xaccel.value , udb_yaccel.value , udb_zaccel.value , udb_xrate.value , udb_yrate.value , udb_zrate.value ) ; 
		udb_serial_start_sending_data();
    }
	else switch ( line_number )
	{
        case 52 :
        {
            line_number ++ ;
            break ;
        }
		case 48 :
		{
			sprintf( debug_buffer , "gyroXoffset, gyroYoffset, gyroZoffset, yawFb, pitchFb, rollFb, pwm1 , pwm2, pwm3, pwm4\r\n" ) ;
            udb_serial_start_sending_data();
			line_number ++ ;
			break ;
		}
		case 38 :
        {
            sprintf(debug_buffer, "X, Y and Z accelerometer offsets: %i, %i, %i\r\n" ,
                XACCEL_OFFSET , YACCEL_OFFSET , ZACCEL_OFFSET    ) ;
            udb_serial_start_sending_data();
            line_number ++ ;
            break ;
        }
		
		case 40 :
		{
			sprintf( debug_buffer , "time,accelOn,launchCount,launched,saturated,rollAngle,rollDeviation,vertX,vertY,vertZ,accX,accY,accZ,gyroX,gyroY,gyroZ, " ) ;
            udb_serial_start_sending_data();
			line_number ++ ;
			break ;
		}
        case 36 :
        {
            sprintf( debug_buffer , "Logging rate is %i lines per second.\r\n",OUTPUT_HZ) ;
            udb_serial_start_sending_data();
            line_number ++ ;
            break ;
        }
#ifdef GAIN_SCHEDULING
        case 32 :
        {
			sprintf( debug_buffer , "Gains schedule is defined in gain_defs.h.\r\n" ) ;
            udb_serial_start_sending_data();
			line_number ++ ;
			break ;            
        }
#endif // GAIN_SCHEDULING
		case 28 :
		{
			sprintf( debug_buffer , "Control mode is %s.\r\n" , CONTROL_TEXT ) ;
            udb_serial_start_sending_data();
			line_number ++ ;
			break ;
		}
        case 24 :
        {
            line_number ++ ;
            break ;
        }
		case 20 :
		{
			sprintf( debug_buffer , "Roll= %i deg, Rate= %i d/s, PWM=%i usecs\r\n" , 
			MAX_ROLL_ANGLE_ , (int16_t) MAX_SPIN_RATE , (int16_t) MAX_SPIN_PULSE_WIDTH ) ;
            udb_serial_start_sending_data();
			line_number ++ ;
			break ;
		}
#ifdef GAIN_SCHEDULING
        case 16 :
        {
            tilt_pwm = gain_defs[gain_print_index].tilt_pwm ;
            max_tilt = gain_defs[gain_print_index].max_tilt ;
            max_tilt_rate = gain_defs[gain_print_index].max_tilt_rate ;
            roll_pwm = gain_defs[gain_print_index].roll_pwm ;
            max_roll = gain_defs[gain_print_index].max_roll ;
            max_roll_rate = gain_defs[gain_print_index].max_roll_rate ;
            gain_time = gain_defs[gain_print_index].time ;
            t_gain_end = gain_time ;
            if ( gain_print_index == 0 )
			{
				sprintf( debug_buffer , "GAIN LIST, tilt_pwm, max_tilt, tilt_rate, roll_pwm, max_roll, roll_rate, t_start, t_end\r\nGAIN DEF,%i,%.1f,%i,%i,%i,%i,%.1f,%.1f\r\n" ,\
                    tilt_pwm,(double)max_tilt,max_tilt_rate,roll_pwm,max_roll,max_roll_rate, ((double)t_gain_start )/((double)10) , ((double)t_gain_end )/((double)10) );
                udb_serial_start_sending_data();
			}
            else
            {
                sprintf( debug_buffer , "GAIN DEF,%i,%.1f,%i,%i,%i,%i,%.1f,%.1f\r\n" ,\
                    tilt_pwm,(double)max_tilt,max_tilt_rate,roll_pwm,max_roll,max_roll_rate, ((double)t_gain_start )/((double)10) , ((double)t_gain_end )/((double)10) );
                udb_serial_start_sending_data();
            }
            if ( gain_print_index < NUM_GAINS - 1)
			{
				gain_print_index ++ ;
                t_gain_start = t_gain_end ;
			}
			else
			{
				line_number ++ ;
			}
			break ;
        }
#endif // GAIN_SCHEDULING 
        case 12 :
        {
#if ( CONTROL_TYPE == TILT_PATTERN )
			int16_t tilt_tilt = tilt_defs[tilt_print_index].tilt ;
			int16_t tilt_b = tilt_defs[tilt_print_index].b ;
			int16_t tilt_x = tilt_defs[tilt_print_index].x ;
			int16_t tilt_y = tilt_defs[tilt_print_index].y ;
            roll_step = tilt_defs[tilt_print_index].roll_step ;
			tilt_t = tilt_defs[tilt_print_index].t ;
			t_end = tilt_t ;
			if ( tilt_print_index == 0 )
			{
				sprintf( debug_buffer , "TILT LIST,tilt,bearing,X,Y,roll_step,start time,end time\r\nTILT DEF,%.1f,%i,%i,%i,%i,%.1f,%.1f\r\n" , ((double)tilt_tilt ) , tilt_b , tilt_x , tilt_y , roll_step , ((double)t_start )/((double)10) , ((double)t_end )/((double)10) );
                udb_serial_start_sending_data();
			}
			else
			{
				sprintf( debug_buffer , "TILT DEF,%.1f,%i,%i,%i,%i,%.1f,%.1f\r\n" , ((double)tilt_tilt ) , tilt_b , tilt_x , tilt_y , roll_step , ((double)t_start )/((double)10) , ((double)t_end )/((double)10) );
                udb_serial_start_sending_data();
			}
			if ( tilt_print_index < NUM_TILTS - 1)
			{
				tilt_print_index ++ ;
				t_start = t_end ;
			}
			else
			{
				line_number ++ ;
			}
			break ;
#else
            line_number ++ ;
            break ;
#endif 
        }    
        case 11 :
        {
            		sprintf( debug_buffer , "Rate= %5.1f d/s, PWM=%i usecs\r\n" ,
                    MAX_TILT_RATE ,(int16_t) MAX_TILT_PULSE_WIDTH 
				) ;
            line_number ++ ;
            udb_serial_start_sending_data();
			break ;
        }
        
        case 10 :
        {       
            sprintf( debug_buffer , "Tilt= %5.1f deg, " ,
			MAX_TILT_ANGLE 
				) ;
            line_number ++ ;
            udb_serial_start_sending_data();
			break ;         
        }
             
        case 9 :
        {
            
            sprintf( debug_buffer , "Gyro range %i DPS, calib %6.4f\r\n" ,
			GYRO_RANGE , CALIBRATION
				) ;
            line_number ++ ;
            udb_serial_start_sending_data();
			break ;
            
        }
		case 8 :
		{
         
			sprintf( debug_buffer , "%s, %s\r\n" ,
			REVISION, DATE 
				) ;
            line_number ++ ;
            udb_serial_start_sending_data();
			break ;
		}
        case 4 :
        {
            line_number ++ ;
            break ;
        }
		case 56 :
		{
			roll_reference.x = rmat[0];
			roll_reference.y = rmat[3];
			roll_angle = rect_to_polar16(&roll_reference) ;
#if(OUTPUT_HZ==10)  
            sprintf(debug_buffer, "%i:%2.2i.%.1i,%i,%i,%i,%.2f,%i,%i,%i,%i,%i,%i,%i,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%i,%i,%i,%i,%i,%i,%i\r\n",
			minutes, seconds , tenths ,  accelOn, launch_count, launched , ((double)roll_angle)/(182.0) , 
#else
            sprintf(debug_buffer, "%i:%2.2i.%.2i,%i,%i,%i,%i,%.2f,%i,%i,%i,%i,%i,%i,%i,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%i,%i,%i,%i,%i,%i,%i\r\n",                  
          	minutes, seconds , hundredths ,  accelOn, launch_count, launched , roll_saturated, ((double)roll_angle)/(182.0) ,           
#endif // OUTPUT_HZ
            roll_deviation,
			rmat[6], rmat[7], rmat[8] ,
			-( udb_xaccel.value)/2 + ( udb_xaccel.offset ) / 2 , 
			( udb_yaccel.value)/2 - ( udb_yaccel.offset ) / 2 ,
			( udb_zaccel.value)/2 - ( udb_zaccel.offset ) / 2 ,
			((double)(  omegaAccum[0])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)(  omegaAccum[1])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)(  omegaAccum[2])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)( (int16_t)(omegagyro_filtered[0].WW>>12))) / ((double)( GYRO_FACTOR*8 )) ,
			((double)( (int16_t)(omegagyro_filtered[1].WW>>12))) / ((double)( GYRO_FACTOR*8 )) ,
			((double)( (int16_t)(omegagyro_filtered[2].WW>>12))) / ((double)( GYRO_FACTOR*8 )) ,
			yaw_feedback_horizontal/2 ,
			pitch_feedback_horizontal/2 ,
			total_roll_feedback_horizontal/2 ,
			udb_pwOut[1]/2 ,
			udb_pwOut[2]/2 ,
			udb_pwOut[3]/2 ,
			udb_pwOut[4]/2 ) ;
//			(uint16_t) udb_cpu_load() );
#if (OUTPUT_HZ == 10)
			tenths ++ ;
#else
			hundredths += 4 ;
#endif // OUTPUT_HZ
#if  (OUTPUT_HZ == 10)          
			if ( tenths == 10 )
#else
			if ( hundredths == 100 )
#endif // OUTPUT_HZ
			{
#if (OUTPUT_HZ == 10)
				tenths = 0 ;
#else
				hundredths = 0 ;
#endif // OUTPUT_HZ
				seconds++ ;
				if ( seconds == 60 )
				{
					seconds = 0 ;
					minutes++ ;
				}
			}
            udb_serial_start_sending_data();
			break ;
		}
        default:
        {
            line_number ++ ;
            break ;
        }
	}
}

// Return one character at a time, as requested.
// Requests will stop after we send back a -1 end-of-data marker.
int16_t udb_serial_callback_get_byte_to_send(void)
{
	uint8_t c = debug_buffer[db_index++];

	if (c == 0) return -1;
	return c;
}

// Don't respond to serial input
void udb_serial_callback_received_byte(uint8_t rxchar)
{
	// Do nothing
}

void udb_callback_radio_did_turn_off(void)
{
}

void compute_control_gains(void)
{
    tilt_allotment_int = 2*tilt_pwm ;
    spin_allotment_int = 2*roll_pwm ;
    total_deflection = tilt_allotment_int + spin_allotment_int ;
    max_roll_angle = max_roll ;
    max_roll_binary = ((int32_t) ((int32_t)286*(int32_t)max_roll ));
    tilt_gain = ( 4.0 * (float)tilt_allotment_int / ( max_tilt / 57.3 ) ) ;
    spin_gain = ( ((float) spin_allotment_int) * ( 256.0 / 65.0) * ( 256.0 / (float) max_roll_rate ) ) ;
    tilt_rate_gain =  (((float)tilt_allotment_int) * ( 256.0 / 65.0) * ( 256.0 / MAX_TILT_RATE ) ) ;
}

#define MAX_FLOAT_U 64000.0 // approximate
#define MAX_FLOAT_S 32000.0 // approximate

int16_t signed_saturate(float x )
{
    if (x>MAX_FLOAT_S)
    {
        return (int16_t) MAX_FLOAT_S ;
    }
    else if (x<-MAX_FLOAT_S)
    {
        return (int16_t) -MAX_FLOAT_S ;
    }
    else
    {
        return (int16_t) x ;
    }
}
uint16_t unsigned_saturate(float x) 
{
    if(x<MAX_FLOAT_U)
    {
        return (uint16_t) x ;
    }
    else
    {
        return (uint16_t) MAX_FLOAT_U ;
    }
}
