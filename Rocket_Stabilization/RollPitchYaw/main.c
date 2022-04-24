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

// legacy #defines:
#define CUSTOM_OFFSETS
#define USE_TILT (0)
#define ROLL_ONLY 1
#define ROLL_PLUS_VERTICAL 2
#define ROLL_PLUS_TILT 3

#define XACCEL_OFFSET	( 0 )
#define YACCEL_OFFSET	( 0 )
#define ZACCEL_OFFSET	( 0 )
#define XRATE_OFFSET	( 0 )
#define YRATE_OFFSET	( 0 )
#define ZRATE_OFFSET	( 0 )

// tilt is not used for now
#define MAX_TILT_ANGLE ( 7.5 ) // degrees
#define MAX_TILT_RATE ( 100.0 ) // degrees per second
#define MAX_TILT_PULSE_WIDTH ( 0.0 ) // microseconds




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

int16_t fail_safe(void);
int16_t dead_band_compensate(int16_t pwm) ;

#define MAX_ROLL_BINARY ((int32_t) ((int32_t)286*(int32_t)MAX_ROLL_ANGLE ))

int16_t apogee = 0 ;
int16_t tilted = 0 ;

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

// Called every 1/40 second at high priority
void udb_heartbeat_40hz_callback(void)
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

//int16_t hundredths = 0 ;
int16_t tenths = 0 ;
int16_t seconds = 0 ;
int16_t minutes = 0 ;

#define TILT_ALLOTMENT ( 2.0*MAX_TILT_PULSE_WIDTH )
#define SPIN_ALLOTMENT ( 2.0*MAX_SPIN_PULSE_WIDTH )
#define TILT_ALLOTMENT_INT (( uint16_t ) TILT_ALLOTMENT)
#define SPIN_ALLOTMENT_INT (( uint16_t ) SPIN_ALLOTMENT)
#define TOTAL_DEFLECTION ( TILT_ALLOTMENT_INT + SPIN_ALLOTMENT_INT )

#define TILT_GAIN ( 4.0 * TILT_ALLOTMENT / ( MAX_TILT_ANGLE / 57.3 ) ) 
#define SPIN_GAIN ( SPIN_ALLOTMENT * ( 256.0 / 65.0) * ( 256.0 / MAX_SPIN_RATE ) )
#define TILT_RATE_GAIN ( TILT_ALLOTMENT * ( 256.0 / 65.0) * ( 256.0 / MAX_TILT_RATE ) )

int16_t offsetX ;
int16_t offsetY ;
int16_t offsetZ ;

struct tilt_def {
	int16_t tilt ;
	int16_t b ;
	int16_t x ;
	int16_t y ;
	uint16_t t	;		
};


uint16_t tilt_index = 0 ;
uint16_t tilt_print_index = 0 ;
int16_t tilt_x ;
int16_t tilt_y ;
uint16_t tilt_t ;

int16_t controlModeYawPitch = YAW_PITCH_ENABLE ;
int16_t controlModeRoll = ROLL_ENABLE ;
int16_t launched = 0 ;
int16_t accelEarthVertical = 0 ;
int32_t velocityEarthVertical = 0 ;
int16_t launch_count = 0 ;
int16_t prelaunch_timer = PRELAUNCH_DELAY ;
int16_t flight_timer = FLIGHT_TIME ;

int16_t lockout = 0 ;
int16_t enable_control = 0 ;

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
	accum.WW = __builtin_mulsu ( tilt , ( uint16_t ) TILT_GAIN ) ;
#if ( GYRO_RANGE == 500 )
	accum.WW += __builtin_mulsu (  tilt_rate , ( uint16_t ) 2*TILT_RATE_GAIN ) ; // 2 is because use of drift corrected values instead of raw values
#elif ( GYRO_RANGE == 1000 )
	accum.WW += __builtin_mulsu (  tilt_rate , ( uint16_t ) 4*TILT_RATE_GAIN ) ; // 1000 degree per second
#else
#error set GYRO_RANGE to 500 or 1000 in options.h
#endif // GYRO_RANGE
	return saturate ( TILT_ALLOTMENT_INT , accum._.W1 )  ;
}

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

	roll_margin_pitch = TOTAL_DEFLECTION - abs ( pitch_feedback ) ;
	roll_margin_yaw = TOTAL_DEFLECTION - abs ( yaw_feedback ) ;
	yaw_margin_minus_pitch_margin_over_2 = ( roll_margin_yaw - roll_margin_pitch ) / 2 ;
	abs_yaw_margin_minus_pitch_margin_over_2 = abs ( yaw_margin_minus_pitch_margin_over_2 ) ;
	
#ifdef NO_MIXING
	net_roll_margin = SPIN_ALLOTMENT_INT ;
#else	
	net_roll_margin = ( roll_margin_pitch + roll_margin_yaw ) / 2 ;
#endif

#if ( GYRO_RANGE == 500 )
	accum.WW = __builtin_mulsu (  roll_rate , ( uint16_t ) 2*SPIN_GAIN*ROLL_RATE_ENABLE ) ; // 2 is because use of drift corrected values instead of raw values
#elif ( GYRO_RANGE == 1000 )
	accum.WW = __builtin_mulsu (  roll_rate , ( uint16_t ) 4*SPIN_GAIN*ROLL_RATE_ENABLE ) ; // 1000 degree per second
#else
#error set GYRO_RANGE to 500 or 1000 in options.h
#endif // GYRO_RANGE
	
	accum._.W1 += __builtin_divsd( __builtin_mulsu( roll_deviation , SPIN_ALLOTMENT_INT ) , MAX_ROLL_ANGLE ) ;
			
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
		if (( launched == 1) && (fail_safe() == 1))
		{
			lockout = 1 ;
		}
		if (GROUND_TEST==1)
		{
			if (prelaunch_timer == 0 )
			{
				launched = 1 ;
			}
			if ( (launched==1)&&(flight_timer>0)&&(lockout==0))
			{
				enable_control = 1 ;
			}
			else
			{
				enable_control = 0 ;
			}
		}
		else
		{
			if ( (launched==1)&&(flight_timer>0))
			{
				enable_control = 1 ;
			}
			else
			{
				enable_control = 0 ;
			}
		}
		
		if (udb_heartbeat_counter % 40 == 0)
		{
			if (prelaunch_timer>0)
			{
				prelaunch_timer = prelaunch_timer - 1 ;
			}
			if (( launched == 1) && (flight_timer>0))
			{
				flight_timer = flight_timer -1 ;
			}
			if (enable_control==1)
			{
				udb_led_toggle(LED_RED) ;
			}
			else
			{
				LED_RED = LED_OFF ;
			}
			if (lockout==0)
			{
				udb_led_toggle(LED_GREEN);
			}
			else
			{
				LED_GREEN = LED_OFF ;
			}
		}
		if ( launched == 1 )
		{
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
			if ( roll_angle_32.WW > MAX_ROLL_BINARY )
			{
				roll_angle_32.WW = MAX_ROLL_BINARY ;
			}
			else if ( roll_angle_32.WW < - MAX_ROLL_BINARY  )
			{
				roll_angle_32.WW = - MAX_ROLL_BINARY ;
			}
			roll_deviation = (int16_t)(roll_angle_32.WW/(int32_t)286);	
		}
		else
		{
			//LED_RED = LED_OFF ;
		}

		{
			offsetX = TILT_X ;
			offsetY = TILT_Y ;
		}
			
		{
			if ( ( _RA2 == 0 ) || ( _RA3 == 0 ) ) // ground test simulate launch 
			{
				launched = 1 ;
			}
		}
	

		if ( ( controlModeYawPitch == 1 )  )
		{
			pitch_feedback_horizontal = tilt_feedback ( rmat[6] - offsetX , -omega[1] ) ;
			yaw_feedback_horizontal = tilt_feedback ( rmat[7] - offsetY , omega[0] ) ;
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
			
		if (enable_control==1)
		{
			udb_pwOut[1] = CENTER POLARITY dead_band_compensate(total_roll_feedback_horizontal) ;
		}
		else
		{
			udb_pwOut[1] = 0 ;
		}
		udb_pwOut[2] = 0 ;
		udb_pwOut[3] = 0 ;
		udb_pwOut[4] = 0 ;
		udb_pwOut[5] = 0 ;
		udb_pwOut[6] = 0 ;
		udb_pwOut[7] = 0 ;
		udb_pwOut[8] = 0 ;
		
		

	}

//	// Serial output at 2Hz  (40Hz / 20)
//	if (udb_heartbeat_counter % 20 == 0)

//  // Serial output at 10Hz
	if (udb_heartbeat_counter % 4 == 0)

//	// Serial output at 20Hz
//	if (udb_heartbeat_counter % 2 == 0)

//	otherwise, Serial output at 40 Hz
	{
		if (dcm_flags._.calib_finished)
		{
			send_debug_line();
		}
	}
}

int16_t fail_safe()
{
	if (( rmat[8]> LOCKOUT_COS_TILT)&&(abs(roll_deviation)<LOCKOUT_ROLL))
	{
		return 0 ;
	}
	else
	{
		return 1 ;
	}
}

int16_t dead_band_memory = 0 ;
int16_t dead_band_compensate ( int16_t pwm_in)
{
	if (abs(pwm_in) < DEAD_BAND)
	{
		if ( pwm_in > 0)
		{
			dead_band_memory = dead_band_memory + pwm_in ;
			if ( dead_band_memory >= DEAD_BAND)
			{
				dead_band_memory = dead_band_memory - DEAD_BAND ;
				return DEAD_BAND ;
			}
			else
			{
				return 0 ;
			}
		}
		else
		{
			dead_band_memory = dead_band_memory + pwm_in ;
			if ( dead_band_memory <= - DEAD_BAND)
			{
				dead_band_memory = dead_band_memory + DEAD_BAND ;
				return - DEAD_BAND ;
			}
			else
			{
				return 0 ;
			}
		}
	}
	else
	{
		return pwm_in ;
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

int16_t line_number = 1 ;
// Prepare a line of serial output and start it sending
void send_debug_line(void)
{
	db_index = 0;
	if( RECORD_OFFSETS == 1 )
	{
		sprintf( debug_buffer , "%i, %i, %i, %i, %i, %i\r\n" , 
			udb_xaccel.value , udb_yaccel.value , udb_zaccel.value , udb_xrate.value , udb_yrate.value , udb_zrate.value ) ; 
	}
	else switch ( line_number )
	{
		
		case 5 :
		{

			{
				sprintf( debug_buffer , "gyroXoffset, gyroYoffset, gyroZoffset, fail_safe, pwm1\r\n" ) ;
			}

			line_number ++ ;
			break ;
		}
		
		case 4 :
		{
			sprintf( debug_buffer , "time, accelOn, launchCount, launched, rollAngle, rollDeviation, vertX, vertY, vertZ, accX, accY, accZ, gyroX, gyroY, gyroZ, " ) ;
			line_number ++ ;
			break ;
		}
		case 3 :
		{
			sprintf(debug_buffer , "PL_delay= %i, FLT_time= %i\r\n" ,
					PRELAUNCH_DELAY , FLIGHT_TIME) ;
			//sprintf( debug_buffer , "Control mode is %s.\r\n" , CONTROL_TEXT ) ;
			line_number ++ ;
			break ;
		}
		case 2 :
		{
			sprintf( debug_buffer , "Roll= %i deg, Rate= %i d/s, PWM=%i usecs\r\n" , 
			MAX_ROLL_ANGLE , (int16_t) MAX_SPIN_RATE , (int16_t) MAX_SPIN_PULSE_WIDTH ) ;
			line_number ++ ;
			break ;
		}
		case 1 :
		{
			sprintf( debug_buffer , "%s, %s\r\nGyro range %i DPS, calib %6.4f\r\nCntr= %i, Dd_bnd= %i, L_O= %i\r\n" ,
			REVISION, DATE, GYRO_RANGE , CALIBRATION ,
			CENTER , DEAD_BAND , LOCKOUT_ROLL
			
			
			 	) ;
			line_number ++ ;
			break ;
		}
		case 6 :
		{
			roll_reference.x = rmat[0];
			roll_reference.y = rmat[3];
			roll_angle = rect_to_polar16(&roll_reference) ;
			sprintf(debug_buffer, "%i:%2.2i.%.1i,%i,%i,%i,%.2f,%i,%i,%i,%i,%i,%i,%i,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%i,%i\r\n",
			minutes, seconds , tenths ,  accelOn, launch_count, launched , ((double)roll_angle)/(182.0) , 
			roll_deviation,
			rmat[6], rmat[7], rmat[8] ,
			-( udb_xaccel.value)/2 + ( udb_xaccel.offset ) / 2 , 
			( udb_yaccel.value)/2 - ( udb_yaccel.offset ) / 2 ,
			( udb_zaccel.value)/2 - ( udb_zaccel.offset ) / 2 ,
			((double)(  omegaAccum[0])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)(  omegaAccum[1])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)(  omegaAccum[2])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)( omegacorrI[0])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)( omegacorrI[1])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)( omegacorrI[2])) / ((double)( GYRO_FACTOR/2 )) ,
			fail_safe() ,
			udb_pwOut[1]/2 ) ;
//			(uint16_t) udb_cpu_load() );
			tenths ++ ;
//			hundredths += 5 ;
			if ( tenths == 10 )
//			if ( hundredths == 100 )
			{
				tenths = 0 ;
//				hundredths = 0 ;
				seconds++ ;
				if ( seconds == 60 )
				{
					seconds = 0 ;
					minutes++ ;
				}
			}
			break ;
		}
	}
	udb_serial_start_sending_data();
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
