// gain definitions
// tilt_pwm , max_tilt , max_tilt_rate , roll_pwm , max_roll , max_roll_rate, time
// tilt_pwm maximum pwm for tilt control in integer microseconds
// max_tilt maximum tilt, float, in degrees
// max_tilt_rate, integer, degrees/sec
// roll_pwm maximum pwm for roll control in integer microseconds
// max_roll maximum roll in integer degrees
// max_roll_rate maximum roll rate in integer degrees
// time, time to trigger next entry, integer tenths of a second

#define GAIN_DEFS {\
{ 250 , 7.5 , 200 , 250 , 360 , 2000 , 34},\
{ 225 , 8.5 , 190 , 225 , 360 , 1900 , 41},\
{ 200 , 9.5 , 180 , 200 , 360 , 1800 , 47},\
{ 175 , 10.5 , 170 , 175 , 360 , 1700 , 61},\
{ 150 , 11.5 , 160 , 150 , 360 , 1600 , 67},\
{ 125 , 12.5 , 150 , 125 , 360 , 1500 , 73},\
{ 100 , 13.5 , 140 , 100 , 360 , 1500 , 80},\
{ 75 , 14.5 , 130 , 75 , 360 , 1300 , 87},\
{ 50 , 15.5 , 120 , 50 , 360 , 1200 , 95},\
{ 0 , 16.5 , 110 , 0 , 360 , 1100 , 994}\
}
