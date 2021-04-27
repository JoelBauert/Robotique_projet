#include <main.h>
#include <distance.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <ch.h>
#include <hal.h>
#include <usbcfg.h>
#include <chprintf.h>

static uint8_t stop; //boolean value to signal that an obstacle is close

uint8_t get_stop(void)
{
	return stop;
}

//tread for handling the IR proximity sensors
static THD_WORKING_AREA(distance_thd_wa, 256);
static THD_FUNCTION(distance_thd, arg)
{
	while(1)
	{
	find_distance();
	}
}

/*
 * function initializing the thread
 */
void distance_start(void)
{
	chThdCreateStatic(distance_thd_wa, sizeof(distance_thd_wa), NORMALPRIO, distance_thd, NULL);
}

/*
 * function to check the 4 front facing proximity sensors and set the stop boolean if the e-puck is close to an obstacle
 */
void find_distance(void)
{
	int d_FR_17, d_FR_49, d_R, d_BR, d_BL, d_L, d_FL_49, d_FL_17;
	calibrate_ir();

	d_FR_17 = get_prox(FR_17); //ir sensor front right 17 degrees
	d_FR_49 = get_prox(FR_49);
	d_FL_17 = get_prox(FL_17);
	d_FL_49 = get_prox(FL_49);

//	chprintf((BaseSequentialStream *)&SD3, "d_FR_17=%d, d_FR_49=%d, d_FL_17=%d, d_FL_49=%d\n", d_FR_17, d_FR_49, d_FL_17, d_FL_49);

	if(d_FR_17>=THRESHOLD_DIST || d_FR_49>=THRESHOLD_DIST || d_FL_17>=THRESHOLD_DIST || d_FL_49>=THRESHOLD_DIST)
	{
		stop = true;
	}
	else
	{
		stop = false;
	}

}
