/* 
 * File:   gyro_offsets_table.h
 * Author: bill
 *
 * Created on December 20, 2022, 9:35 AM
 */
#include  "options.h"
#include <stdint.h>
#include "../libDCM/libDCM.h"
#include "../libDCM/libDCM_defines.h"
#include "../libDCM/mathlibNAV.h"
//#include "../libDCM/gpsData.h"
//#include "../libDCM/gpsParseCommon.h"
#include "../libDCM/rmat.h"
#include "../libUDB/heartbeat.h"
//#include "../libUDB/serialIO.h"
//#include "../libUDB/servoOut.h"
#include "../libUDB/ADchannel.h"
#include <math.h>  

#ifdef GYRO_OFFSET_TABLE

typedef struct gyro_offset_table_entry { int16_t x ; int16_t y ; int16_t z ; } gyro_offset_table_entry ;

#include GYRO_OFFSET_TABLE

#if (STEP_SIZE == 1024)
#define LOOKUP_LSB_MASK 0x03FF
#define MSB_SHIFT 10
#elif ( STEP_SIZE == 256 )
#define LOOKUP_LSB_MASK 0x00FF
#define MSB_SHIFT 8
#elif ( STEP_SIZE == 64 )
#define LOOKUP_LSB_MASK 0x003F
#define MSB_SHIFT 6
#else
#error "unsupported or undefine STEP_SIZE"
#endif // STEP_SIZE

extern struct ADchannel mpu_temp;

int16_t temperature_index ;

int16_t gyro_offset[3] ;


#ifdef X_CROSS_COUPLING
#if ( X_CROSS_COUPLING == 0 )
#warning "X_CROSS_COUPLING is 0"
#endif 
int16_t cross_coupling = X_CROSS_COUPLING ;
#else
#warning "X_CROSS_COUPLING is not defined"
int16_t cross_coupling = 0 ;
#endif

uint16_t index_msb = 0 ;
uint16_t index_lsb = 0 ;
int16_t left_entry[3];
int16_t right_minus_left[3];
uint16_t number_entries ;
int16_t accel_gyro_coupling_compensation[]= { 0 , 0 , 0 };

#if (ACCEL_RANGE == 2)
#define CROSS_SHIFT 13
#elif (ACCEL_RANGE == 4)
#define CROSS_SHIFT 12
#elif (ACCEL_RANGE == 8)
#define CROSS_SHIFT 11
#elif (ACCEL_RANGE == 16)
#define CROSS_SHIFT 10
#endif // ACCEL_RANGE

void lookup_gyro_offsets(void)
{ 
	temperature_index = mpu_temp.value - TABLE_ORIGIN ;
	if (temperature_index < 0)
	{
		index_msb = 0 ;
		index_lsb = 0 ;
		gyro_offset[0] = residual_offset[0]+ gyro_offset_table[0].x ;
		gyro_offset[1] = residual_offset[1]+ gyro_offset_table[0].y ;
		gyro_offset[2] = residual_offset[2]+ gyro_offset_table[0].z ;
	}
	else
	{
		index_lsb = temperature_index & LOOKUP_LSB_MASK ;
		index_msb = temperature_index >> MSB_SHIFT ; 
		number_entries = (sizeof (gyro_offset_table))/(sizeof (gyro_offset_table_entry)) ;
		if ( index_msb >= (number_entries - 1 ))
		{
			gyro_offset[0] = residual_offset[0]+ gyro_offset_table[number_entries - 1].x ;
			gyro_offset[1] = residual_offset[1]+ gyro_offset_table[number_entries - 1].y ;
			gyro_offset[2] = residual_offset[2]+ gyro_offset_table[number_entries - 1].z ;
		}
		else
		{
			left_entry[0]= gyro_offset_table[index_msb].x ;
			left_entry[1]= gyro_offset_table[index_msb].y ;
			left_entry[2]= gyro_offset_table[index_msb].z ;
			
			right_minus_left[0]= gyro_offset_table[index_msb+1].x - left_entry[0] ;
			right_minus_left[1]= gyro_offset_table[index_msb+1].y - left_entry[1] ;
			right_minus_left[2]= gyro_offset_table[index_msb+1].z - left_entry[2] ;
			
			gyro_offset[0] = accel_gyro_coupling_compensation[0]
                    +residual_offset[0] 
					+ left_entry[0] 
					+ __builtin_divsd(__builtin_mulss(right_minus_left[0],index_lsb),STEP_SIZE);
			gyro_offset[1] = accel_gyro_coupling_compensation[1]
                    +residual_offset[1] 
					+ left_entry[1] 
					+ __builtin_divsd(__builtin_mulss(right_minus_left[1],index_lsb),STEP_SIZE);
			gyro_offset[2] = accel_gyro_coupling_compensation[2]
                    +residual_offset[2] 
					+ left_entry[2] + 
					__builtin_divsd(__builtin_mulss(right_minus_left[2],index_lsb),STEP_SIZE);
		}
	}
    udb_xrate.offset = (gyro_offset[0])>>6 ;
	udb_yrate.offset = (gyro_offset[1])>>6 ;
	udb_zrate.offset = (gyro_offset[2])>>6 ;
}

#ifdef ACCEL_TABLE

int16_t accel_offset[3] ;

void lookup_accel_offsets(void)
{
  	temperature_index = mpu_temp.value - ACCEL_TABLE_ORIGIN ;
	if (temperature_index < 0)
	{
		index_msb = 0 ;
		index_lsb = 0 ;
		accel_offset[0] = accel_residual_offset[0]+ accel_offset_table[0].x ;
        accel_offset[1] = accel_residual_offset[1]+ accel_offset_table[0].y ;
        accel_offset[2] = accel_residual_offset[2]+ accel_offset_table[0].z ;

	}
	else
	{
		index_lsb = temperature_index & LOOKUP_LSB_MASK ;
		index_msb = temperature_index >> MSB_SHIFT ; 
		number_entries = (sizeof (accel_offset_table))/(sizeof (gyro_offset_table_entry)) ;
		if ( index_msb >= (number_entries - 1 ))
		{
			accel_offset[0] = accel_residual_offset[0]+ accel_offset_table[number_entries - 1].x ;
            accel_offset[1] = accel_residual_offset[1]+ accel_offset_table[number_entries - 1].y ;
			accel_offset[2] = accel_residual_offset[2]+ accel_offset_table[number_entries - 1].z ;
				
        }
		else
		{
			left_entry[0]= accel_offset_table[index_msb].x ;
			left_entry[1]= accel_offset_table[index_msb].y ;
			left_entry[2]= accel_offset_table[index_msb].z ;
			
			right_minus_left[0]= accel_offset_table[index_msb+1].x - left_entry[0] ;
			right_minus_left[1]= accel_offset_table[index_msb+1].y - left_entry[1] ;
			right_minus_left[2]= accel_offset_table[index_msb+1].z - left_entry[2] ;
			
			accel_offset[0] = 
                    accel_residual_offset[0] 
					+ left_entry[0] 
					+ __builtin_divsd(__builtin_mulss(right_minus_left[0],index_lsb),STEP_SIZE);
			accel_offset[1] = 
                    accel_residual_offset[1] 
					+ left_entry[1] 
					+ __builtin_divsd(__builtin_mulss(right_minus_left[1],index_lsb),STEP_SIZE);
			accel_offset[2] = 
                    accel_residual_offset[2] 
					+ left_entry[2] 
					+ __builtin_divsd(__builtin_mulss(right_minus_left[2],index_lsb),STEP_SIZE);
			
		}
	} 
    udb_xaccel.offset = accel_offset[0] ;
	udb_yaccel.offset = accel_offset[1] ;
	udb_zaccel.offset = accel_offset[2] ;
}

#else

int16_t accel_residual_offset[] = { 0 , 0 , 0 } ;

#endif // ACCEL_TABLE

int64_t samples_64t = 0 ;
int32_t samples_32t = 0 ;
int16_t gyro_offset_entry[] = { 0 , 0 , 0 } ;
int64_t xx_sum = 0 ;
int64_t xy_sum[] = { 0 , 0 , 0  } ;
int32_t x_sum = 0 ;
int32_t y_sum[] = { 0 , 0 , 0  } ;
int32_t xx_bar = 0 ;
int32_t xy_bar[] = { 0 , 0 , 0  } ;
int16_t x_bar = 0 ;
int16_t y_bar[] = { 0 , 0 , 0  } ;
int64_t xx_bar_minus_x_bar_x_bar ;
int16_t offset_left[3] ;
int16_t offset_right[3] ;
int16_t offset_previous[3];

int16_t adjusted_temperature = 0 ;
int16_t temperature_offset = 0 ;
int16_t initial_temperature = 0 ;
int16_t reported_temperature = -8000 ;

int16_t initial_temp_recorded = 0 ;
int16_t initial_temp_reported = 0 ;

#else
void lookup_accel_offsets(void)
{
    
}
void lookup_gyro_offsets(void)
{
    
}
#endif // GYRO_OFFSET_TABLE