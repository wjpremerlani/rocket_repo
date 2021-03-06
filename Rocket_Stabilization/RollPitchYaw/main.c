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

char debug_buffer[512];
int db_index = 0;
void send_debug_line(void);

extern int16_t omega[] ;
extern int16_t omegacorrI[] ;

struct relative2D roll_reference ;
int16_t roll_angle ;
int16_t rect_to_polar16(struct relative2D *xy);

int16_t apogee = 0 ;
int16_t tilted = 0 ;

#define RECORD_OFFSETS	( 0 ) // set to 1 in order to record accelerometer and gyro offsets in telemetry

int main(void)
{
	_TRISA2 = 1 ; // SCL is input pin for enabling yaw/pitch control
	_TRISA3 = 1 ; // SDA is input pin for enabling roll control
	_TRISD15 = 0 ; // repurpose PWM8 input as an output
	_LATD15 = 0 ;
	mcu_init();

	// Set up the libraries
	udb_init();
	dcm_init();

	udb_serial_set_rate(SERIAL_BAUDRATE);

	LED_GREEN = LED_OFF;

	// Start it up!
	while (1)
	{
		udb_run();
	}

	return 0;
}

void init_events(void)
{
}

// Called every 1/40 second at high priority
void udb_heartbeat_40hz_callback(void)
{
	static int count = 0;

	if (!dcm_flags._.calib_finished)
	{
		// If still calibrating, blink RED
		if (++count > 20)
		{
			count = 0;
			udb_led_toggle(LED_RED);
		}
	}
	else
	{
		// No longer calibrating: solid RED and send debug output
		LED_RED = LED_ON;
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
#define DISTURBANCE ( 0 ) // no disturbance
//#define DISTURBANCE ( 1430 ) // 5 degrees
//#define DISTURBANCE ( 8192 ) // 30 degrees for testing

int16_t offsetZ ;
int16_t offsetX ;

int16_t controlModeYawPitch = 0 ;
int16_t controlModeRoll = 0 ;
int16_t launched = 0 ;
int16_t accelEarthVertical = 0 ;
int32_t velocityEarthVertical = 0 ;
int16_t launch_count = 0 ;

int16_t roll_feedback_vertical_pitch = 0 ;
int16_t roll_feedback_vertical_yaw = 0 ;
int16_t total_roll_feedback_vertical = 0 ;

int16_t pitch_feedback_vertical = 0 ;
int16_t yaw_feedback_vertical = 0 ;

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

int16_t tilt_feedback ( int16_t tilt )
{
	union longww accum ;
	accum.WW = __builtin_mulsu ( tilt , ( uint16_t ) TILT_GAIN ) ;
	return saturate ( TILT_ALLOTMENT_INT , accum._.W1 )  ;
}

void roll_feedback ( int16_t pitch_feedback , int16_t yaw_feedback ,  int16_t roll_rate , 
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
	accum.WW = __builtin_mulsu (  roll_rate , ( uint16_t ) 2*SPIN_GAIN ) ; // 2 is because use of drift corrected values instead of raw values
#elif ( GYRO_RANGE == 1000 )
	accum.WW = __builtin_mulsu (  roll_rate , ( uint16_t ) 4*SPIN_GAIN ) ; // 1000 degree per second
#else
#error set GYRO_RANGE to 500 or 1000 in options.h
#endif // GYRO_RANGE

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
	if (!dcm_flags._.calib_finished)
	{

	}
	else
	{
//		union longww accum;
/*
//		legacy code for introducing an offset
		if ( _RA3 == 0 )
		{
			LED_ORANGE = LED_ON ;
			accum.WW = ( __builtin_mulsu( rmat[5] , DISTURBANCE ) << 2 ) ;
			offsetZ = accum._.W1 ;
			accum.WW = ( __builtin_mulsu( rmat[3] , DISTURBANCE ) << 2 ) ;
			offsetX = accum._.W1 ;
		}
		else
		{
			LED_ORANGE = LED_OFF ;
			offsetX = 0 ;
			offsetZ = 0 ;
		}
*/

		offsetX = 0 ;
		offsetZ = 0 ;	

		if ( _RA2 == 0 ) // check control enable pin
		{
			LED_BLUE = LED_ON ;
			controlModeYawPitch = 1 ;
		}
		else
		{
			LED_BLUE = LED_OFF ;
			controlModeYawPitch = 0 ;
		}

		if ( _RA3 == 0 ) // check control enable pin
		{
			LED_ORANGE = LED_ON ;
			controlModeRoll = 1 ;
		}
		else
		{
			LED_ORANGE = LED_OFF ;
			controlModeRoll = 0 ;
		}

		if ( ( controlModeRoll + controlModeYawPitch ) > 0 )
		{
			_LATD15 = 1 ; // turn on external LED
		}
		else
		{
			_LATD15 = 0 ; // turn on external LED			
		}

		if ( GROUND_TEST == 1 )
		{
			if ( ( _RA3 == 0 ) && ( _RA2 == 0 ) ) // ground test simulate launch by enabling both control modes
			{
				launched = 1 ;
			}
		}
	
#ifdef DETECT_APOGEE // detect apogee by looking at tilt, is there more than 60 degrees
#if  ( MOUNT_ORIENTATION == VERTICAL_MOUNT )
		if ( -rmat[7] < 8256 )
		{
			LED_GREEN = LED_OFF ;
			tilted = 1 ;
		}
		else
		{
			LED_GREEN = LED_ON ;
			tilted = 0 ;
		}
#elif ( MOUNT_ORIENTATION == HORIZONTAL_MOUNT )
		if ( rmat[8] < 8256 )
		{
			LED_GREEN = LED_ON ;
			tilted = 1 ;
		}
		else
		{
			LED_GREEN = LED_OFF ;
			tilted = 0 ;
		}
#endif // MOUNT_ORIENTATION
		if ( ( tilted == 1 ) && ( launched == 1 ) ) // lock in apogee if tilted after launch
		{
			apogee = 1 ;
		}
#endif // DETECT_APOGEE

		if ( ( controlModeYawPitch == 1 ) && ( apogee == 0 ) )
		{

			pitch_feedback_vertical = tilt_feedback ( -rmat[8] ) ;
			yaw_feedback_vertical = tilt_feedback ( rmat[6] ) ;

			pitch_feedback_horizontal = tilt_feedback ( rmat[6] ) ;
			yaw_feedback_horizontal = tilt_feedback ( rmat[7] ) ;

		}

		else
		{
			pitch_feedback_vertical = 0 ;
			yaw_feedback_vertical = 0 ;	

			pitch_feedback_horizontal = 0 ;
			yaw_feedback_horizontal = 0 ;	
		
		}

		
		if ( ( controlModeRoll == 1 )&& ( apogee == 0 ) )
		{

//			roll_feedback ( pitch_feedback_vertical , yaw_feedback_vertical ,  - remove_offset ( udb_yrate.value , udb_yrate.offset ) , 
//							&roll_feedback_vertical_pitch , &roll_feedback_vertical_yaw , &total_roll_feedback_vertical ) ;
//
//			roll_feedback ( pitch_feedback_horizontal , yaw_feedback_horizontal ,  remove_offset ( udb_zrate.value , udb_zrate.offset )  , 
//							&roll_feedback_horizontal_pitch , &roll_feedback_horizontal_yaw , &total_roll_feedback_horizontal ) ;

			roll_feedback ( pitch_feedback_vertical , yaw_feedback_vertical ,   omega[1] , 
							&roll_feedback_vertical_pitch , &roll_feedback_vertical_yaw , &total_roll_feedback_vertical ) ;

			roll_feedback ( pitch_feedback_horizontal , yaw_feedback_horizontal , - omega[2]  , 
							&roll_feedback_horizontal_pitch , &roll_feedback_horizontal_yaw , &total_roll_feedback_horizontal ) ;

		}

		else
		{
			roll_feedback_vertical_pitch = 0 ;
			roll_feedback_vertical_yaw = 0 ;
			total_roll_feedback_vertical = 0 ;

			roll_feedback_horizontal_pitch = 0 ;
			roll_feedback_horizontal_yaw = 0 ;
			total_roll_feedback_horizontal = 0 ;
		}
#ifdef NO_MIXING
#if ( MOUNT_ORIENTATION == 	VERTICAL_MOUNT	)
		udb_pwOut[1] = pitch_feedback_vertical + 3000 ;
		udb_pwOut[2] = yaw_feedback_vertical + 3000 ;
		udb_pwOut[3] = total_roll_feedback_vertical + 3000 ;
#else
		udb_pwOut[1] = pitch_feedback_horizontal + 3000 ;
		udb_pwOut[2] = yaw_feedback_horizontal + 3000 ;
		udb_pwOut[3] = total_roll_feedback_horizontal + 3000 ;
#endif
		udb_pwOut[4] = 3000 ;

		udb_pwOut[5] = 3000 ;
		udb_pwOut[6] = 3000 ;
		udb_pwOut[7] = 3000 ;
		udb_pwOut[8] = 3000 ;		
#else
		udb_pwOut[1] = roll_feedback_vertical_pitch + pitch_feedback_vertical + 3000 ;
		udb_pwOut[2] = roll_feedback_vertical_yaw + yaw_feedback_vertical + 3000 ;
		udb_pwOut[3] = roll_feedback_vertical_pitch - pitch_feedback_vertical + 3000 ;
		udb_pwOut[4] = roll_feedback_vertical_yaw - yaw_feedback_vertical + 3000 ;

		udb_pwOut[5] = roll_feedback_horizontal_pitch + pitch_feedback_horizontal + 3000 ;
		udb_pwOut[6] = roll_feedback_horizontal_yaw + yaw_feedback_horizontal + 3000 ;
		udb_pwOut[7] = roll_feedback_horizontal_pitch - pitch_feedback_horizontal + 3000 ;
		udb_pwOut[8] = roll_feedback_horizontal_yaw - yaw_feedback_horizontal + 3000 ;
#endif
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

int16_t firstline = 1 ;
int16_t secondline = 0 ;
int16_t thirdline = 0 ;
int16_t fourthline = 0 ;
int16_t accelOn ;


#if ( GYRO_RANGE == 500 )
#define GYRO_FACTOR ( 65 ) // UDB5 sensitivity
#elif ( GYRO_RANGE == 1000 )
#define GYRO_FACTOR ( 32 ) // UDB5 sensitivity
#else
#error GYRO_RANGE not specified as 500 or 1000
#endif // GYRO_RANGE

// Prepare a line of serial output and start it sending
void send_debug_line(void)
{
	db_index = 0;
	if( RECORD_OFFSETS == 1 )
	{
//		int16_t gravity2x = (int16_t) 2*GRAVITY ;
//		sprintf( debug_buffer , "%i, %i, %i, %i, %i, %i, %i\r\n" , 
//			gravity2x, udb_xaccel.value , udb_yaccel.value , udb_zaccel.value , udb_xrate.value , udb_yrate.value , udb_zrate.value ) ; 
		sprintf( debug_buffer , "%i, %i, %i, %i, %i, %i\r\n" , 
			udb_xaccel.value , udb_yaccel.value , udb_zaccel.value , udb_xrate.value , udb_yrate.value , udb_zrate.value ) ; 
	}
	else if ( fourthline == 1 )
	{
		fourthline = 0 ;
		if ( GROUND_TEST == 1)
		{
			sprintf( debug_buffer , "gyroXoffset, gyroYoffset, gyroZoffset,yawFbVert, pitchFbVert, rollFbVert, yawFbHoriz, pitchFbHoriz, rollFbHoriz\r\n" ) ;
		}
		else
		{
			sprintf( debug_buffer , "yawFbVert, pitchFbVert, rollFbVert, yawFbHoriz, pitchFbHoriz, rollFbHoriz, out1, out2, out3, out4, out5, out6, out7, out8\r\n" ) ;
		}
	}
	else if ( thirdline == 1 )
	{
		thirdline = 0 ;
		fourthline = 1 ;
		sprintf( debug_buffer , "time, cntlModeYwPtch, cntlModeRoll, accelOn, launchCount, launched, apogee , rollAngle, vertX, vertY, vertZ, accX, accY, accZ, gyroX, gyroY, gyroZ, " ) ;
	}
	else if ( secondline == 1)
	{
		secondline = 0 ;
		thirdline = 1 ;
//		sprintf( debug_buffer , "JJBrd1, rev13, 5/10/2015\r\nTiltMultiplier: %i, RollMultiplier: %i\r\n" , BOARD, REVISION, DATE, (int16_t) TILT_GAIN , (int16_t) SPIN_GAIN ) ;
//		sprintf( debug_buffer , "%s, %s, %s\r\nTiltMultiplier: %i, RollMultiplier: %i\r\nSensorOffsets, Accel: , %i, %i, %i, Gyro: , %i, %i, %i\r\n" , 
		sprintf( debug_buffer , "Offsets, Accel: , %i, %i, %i, Gyro: , %i, %i, %i\r\n" , 
			udb_xaccel.offset , udb_yaccel.offset , udb_zaccel.offset ,
			udb_xrate.offset , udb_yrate.offset , udb_zrate.offset
			 	) ;
	}
	else if ( firstline == 1)
	{
		firstline = 0 ;
		secondline = 1 ;
//		sprintf( debug_buffer , "JJBrd1, rev13, 5/10/2015\r\nTiltMultiplier: %i, RollMultiplier: %i\r\n" , BOARD, REVISION, DATE, (int16_t) TILT_GAIN , (int16_t) SPIN_GAIN ) ;
//		sprintf( debug_buffer , "%s, %s, %s\r\nTiltMultiplier: %i, RollMultiplier: %i\r\nSensorOffsets, Accel: , %i, %i, %i, Gyro: , %i, %i, %i\r\n" , 
		sprintf( debug_buffer , "%s, %s, %s\r\nGyro Range %i DPS\r\nTilt %5.1f deg %i usecs.\r\nSpin %i deg/sec %i usecs.\r\n" ,
			BOARD, REVISION, DATE, GYRO_RANGE ,
			MAX_TILT_ANGLE , (int16_t) MAX_TILT_PULSE_WIDTH , (int16_t) MAX_SPIN_RATE , (int16_t) MAX_SPIN_PULSE_WIDTH 
			//(int16_t) TILT_GAIN , (int16_t) SPIN_GAIN ,
			
			 	) ;
	}
	else
	{
		roll_reference.x = rmat[0];
		roll_reference.y = rmat[3];
		roll_angle = rect_to_polar16(&roll_reference) ;
//		sprintf(debug_buffer, "%i:%2.2i.%.1i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\r\n",
#if ( GROUND_TEST == 0 )
			sprintf(debug_buffer, "%i:%2.2i.%.1i, %i, %i, %i, %i, %i, %i, %.2f, %i, %i, %i, %i, %i, %i, %.2f, %.2f, %.2f, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\r\n",
#else
			sprintf(debug_buffer, "%i:%2.2i.%.1i, %i, %i, %i, %i, %i, %i, %.2f, %i, %i, %i, %i, %i, %i, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %i, %i, %i, %i, %i, %i\r\n",
#endif // GROUND_TEST
			minutes, seconds , tenths ,  controlModeYawPitch, controlModeRoll , accelOn, launch_count, launched , apogee, ((double)roll_angle)/(182.0) ,
			rmat[6], rmat[7], rmat[8] ,
//			offsetX , offsetZ ,
			-( udb_xaccel.value)/2 + ( udb_xaccel.offset ) / 2 , 
			( udb_yaccel.value)/2 - ( udb_yaccel.offset ) / 2 ,
			( udb_zaccel.value)/2 - ( udb_zaccel.offset ) / 2 ,
			((double)(  omegaAccum[0])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)(  omegaAccum[1])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)(  omegaAccum[2])) / ((double)( GYRO_FACTOR/2 )) ,
#if ( GROUND_TEST == 1 )
			((double)( omegacorrI[0])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)( omegacorrI[1])) / ((double)( GYRO_FACTOR/2 )) ,
			((double)( omegacorrI[2])) / ((double)( GYRO_FACTOR/2 )) ,
#endif // GROUND_TEST
			yaw_feedback_vertical ,
			pitch_feedback_vertical ,
			total_roll_feedback_vertical ,
			yaw_feedback_horizontal ,
			pitch_feedback_horizontal ,
#if ( GROUND_TEST == 0 )
			total_roll_feedback_horizontal ,
			udb_pwOut[1] ,
			udb_pwOut[2] ,
			udb_pwOut[3] ,
			udb_pwOut[4] ,
			udb_pwOut[5] ,
			udb_pwOut[6] ,
			udb_pwOut[7] ,
			udb_pwOut[8] ) ;
#else
			total_roll_feedback_horizontal ) ;
#endif // GROUND_TEST
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

void osd_init(void)
{
}
