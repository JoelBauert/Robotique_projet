#include <distance.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <ch.h>
#include <hal.h>
#include <usbcfg.h>
#include <chprintf.h>

static THD_WORKING_AREA(distance_thd_wa, 256);
static THD_FUNCTION(distance_thd, arg)
{
	left_motor_set_speed(300);
	right_motor_set_speed(300);
	while(1)
	{
	find_distance();
	}
}

void distance_start(void)
{
	chThdCreateStatic(distance_thd_wa, sizeof(distance_thd_wa), NORMALPRIO, distance_thd, NULL);
}

void find_distance(void)
{
	int d_FR_17, d_FR_49, d_R, d_BR, d_BL, d_L, d_FL_49, d_FL_17;
	calibrate_ir();

	d_FR_17 = get_prox(FR_17);
	d_FR_49 = get_prox(FR_49);
	d_FL_17 = get_prox(FL_17);
	d_FL_49 = get_prox(FL_49);

	chprintf((BaseSequentialStream *)&SD3, "d_FR_17=%d, d_FR_49=%d, d_FL_17=%d, d_FL_49=%d\n", d_FR_17, d_FR_49, d_FL_17, d_FL_49);

	if(d_FR_17>=THRESHOLD_DIST || d_FR_49>=THRESHOLD_DIST || d_FL_17>=THRESHOLD_DIST || d_FL_49>=THRESHOLD_DIST)
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	else
	{
		left_motor_set_speed(300);
		right_motor_set_speed(300);
	}

}
