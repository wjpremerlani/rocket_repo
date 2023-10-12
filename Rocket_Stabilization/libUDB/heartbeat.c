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


#include "libUDB_internal.h"
#include "oscillator.h"
#include "interrupt.h"
#include "heartbeat.h"

int one_hertz_flag = 0;
uint16_t udb_heartbeat_counter = 0;
#define HEARTBEAT_MAX 57600 // Evenly divisible by many common values: 2^8 * 3^2 * 5^2

static void pulse(void);    // forward declaration

//#define HEARTBEAT_FREQ(x) (udb_heartbeat_counter % (HEARTBEAT_HZ/x) == 0)
//#define HEARTBEAT_CHK(x) (udb_heartbeat_counter % (HEARTBEAT_HZ/x) == 0)

inline boolean heartbeat_chk(uint16_t freq)
{
	return (udb_heartbeat_counter % (HEARTBEAT_HZ/freq) == 0);
//	return HEARTBEAT_CHK(freq);
}

inline uint16_t heartbeat_cnt(void)
{
	return udb_heartbeat_counter;
}

inline void heartbeat(void) // called from ISR
{
	// Start the sequential servo pulses at frequency SERVO_HZ
	if (udb_heartbeat_counter % (HEARTBEAT_HZ/SERVO_HZ) == 0)
	{
		start_pwm_outputs();
	}

	// Capture cpu_timer once per second.
	if (udb_heartbeat_counter % (HEARTBEAT_HZ/1) == 0)
	{
		cpu_load_calc();
		one_hertz_flag = 1;
	}

	// TODO: determine why this is called from the high priority interrupt handler? is it req?
	// Call the periodic callback at 50 Hz
	if (udb_heartbeat_counter % (HEARTBEAT_HZ/50) == 0)
	{
		udb_heartbeat_50hz_callback(); // this was called udb_background_callback_periodic()
	}

	// Trigger the HEARTBEAT_HZ calculations, but at a lower priority
//	_T6IF = 1;
	udb_background_trigger_pulse(&pulse);

	udb_heartbeat_counter = (udb_heartbeat_counter+1) % HEARTBEAT_MAX;
}

// Executes whatever lower priority calculation needs to be done every heartbeat (default: 25 milliseconds)
// This is a good place to eventually compute pulse widths for servos.
static void pulse(void)
{
//	LED_BLUE = LED_OFF;     // indicates logfile activity


	vref_adj = 0;
	udb_callback_read_sensors();
	udb_flags._.a2d_read = 1; // signal the A/D to start the next summation

	// process sensor data, run flight controller, generate outputs. implemented in libDCM.c
	udb_heartbeat_callback(); // this was called udb_servo_callback_prepare_outputs()
}
