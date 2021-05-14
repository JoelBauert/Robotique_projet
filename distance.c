/*
 * distance.c
 *
 *  Created on: 22 avr. 2021
 *      Author: Jessica, Joel
 */
#include <main.h>
#include <distance.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <ch.h>
#include <hal.h>
#include <usbcfg.h>
#include <chprintf.h>

#define FR_17 0 // front right 17'
#define FR_49 1 // front right 49'
#define RIGHT 2 // right
#define BR    3 // back right
#define BL    4 // back left
#define LEFT  5 // left
#define FL_49 6 // front left 49'
#define FL_17 7 // front right 17'

#define THRESHOLD_DIST 200 //distance at which the e-puck should stop

static uint8_t stop; //boolean value to signal that an obstacle is close

uint8_t get_stop(void)
{
	return stop;
}

//tread for handling the IR proximity sensors
static THD_WORKING_AREA(distance_thd_wa, 256);
static THD_FUNCTION(distance_thd, arg)
{
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	systime_t time;
	calibrate_ir();
	while(1)
	{
		time = chVTGetSystemTime();
		find_distance();
		chThdSleepUntilWindowed(time, time+MS2ST(10)); //reduced the sample rate to 100Hz
	}
}

/*
 * @brief: function initializing the thread
 */
void distance_start(void)
{
	chThdCreateStatic(distance_thd_wa, sizeof(distance_thd_wa), NORMALPRIO, distance_thd, NULL);
}

/*
 * @brief: function to check the 4 front facing proximity sensors and set the stop boolean if the e-puck is close to an obstacle
 */
void find_distance(void)
{
	int d_FR_17, d_FR_49, d_FL_49, d_FL_17;

	d_FR_17 = get_prox(FR_17);
	d_FR_49 = get_prox(FR_49);
	d_FL_17 = get_prox(FL_17);
	d_FL_49 = get_prox(FL_49);

	if(d_FR_17>=THRESHOLD_DIST || d_FR_49>=THRESHOLD_DIST || d_FL_17>=THRESHOLD_DIST || d_FL_49>=THRESHOLD_DIST)
		stop = true;
	else
		stop = false;
}
