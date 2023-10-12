// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
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


#include "libDCM_internal.h"
#include "../libUDB/heartbeat.h"
#include "mathlibNAV.h"
#include "rmat.h"


union dcm_fbts_word dcm_flags;

// Calibrate for 10 seconds before moving servos

#define CALIB_COUNT  100    // 2 seconds at 50 Hz
#define GPS_COUNT    1250   // 25 seconds at 50 Hz


extern int16_t rmat[9] ;

void dcm_init(void)
{
	dcm_flags.W = 0;
	dcm_flags._.first_mag_reading = 1;
	dcm_init_rmat();
}

extern inline void read_accel(void) ;

void dcm_align_tilt(void)
{
	int16_t Z ;
	int16_t one_plus_Z ;
	read_accel() ;
	vector3_normalize( &rmat[6] , gplane ) ;
	rmat[2] = -rmat[6];
	rmat[5] = -rmat[7];
	Z = rmat[8];
	one_plus_Z = RMAX + Z ;
	if ( one_plus_Z > 0)
	{
		rmat[0] = RMAX - __builtin_divsd( __builtin_mulss( rmat[6], rmat[6]), one_plus_Z ) ;
		rmat[4] = RMAX - __builtin_divsd( __builtin_mulss( rmat[7], rmat[7]), one_plus_Z ) ;
		rmat[1] = - __builtin_divsd( __builtin_mulss( rmat[6], rmat[7]), one_plus_Z ) ;
		rmat[3] = rmat[1] ;	
	}
	else
	{
		rmat[0] = Z ;
		rmat[4] = Z ;
		rmat[1] = 0 ;
		rmat[3] = 0 ;
	}
	
	
}

void dcm_run_init_step(uint16_t count)
{
	if (count == CALIB_COUNT)
	{
		// Finish calibration
		DPRINT("calib_finished\r\n");
		dcm_flags._.calib_finished = 1;
		dcm_calibrate();
		dcm_align_tilt();
	}
}

// Called at HEARTBEAT_HZ
void udb_heartbeat_callback(void)
{

//  when we move the IMU step to the MPU call back, to run at 200 Hz, remove this
	if (dcm_flags._.calib_finished)
	{
		dcm_run_imu_step();
	}

	dcm_heartbeat_callback();    // this was called dcm_servo_callback_prepare_outputs();

	if (!dcm_flags._.init_finished)
	{
		if (udb_heartbeat_counter % (HEARTBEAT_HZ / 50) == 0)
		{
			dcm_run_init_step(udb_heartbeat_counter / (HEARTBEAT_HZ / 50));
		}
	}

}

// dcm_calibrate is called twice during the startup sequence.
// Firstly 10 seconds after startup, then immediately before the first waggle, which is 10 seconds after getting radio link.  
// This makes sure we get initialized when there's no radio, or when bench testing, 
// and 2nd time is at a time the user knows to keep the plane steady before a flight.
void dcm_calibrate(void)
{
	// Don't allow re/calibrating before the initial calibration period has finished
	if (dcm_flags._.calib_finished)
	{
		udb_a2d_record_offsets();
	}
}
