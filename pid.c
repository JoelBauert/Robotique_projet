/*
 * pid.c
 *
 *  Created on: 27 avr. 2021
 *      Author: jessica
 */
#include <pid.h>
#include <math.h>

static float Kp, Ki, Kd;

void set_pid_param(float kp, float ki, float kd)
{
	Kp = kp;
	Ki = ki;
	Kd = kd;
}

float calcul_pid(float val1, float val2, float threshold, float max)
{
	PID_obj pid;
	float speed;
	static float integral = 0;
	static float previous_error = 0;

	float error = val1-val2;

	if(fabs(error) <= threshold)
	{
		integral = 0;
		speed = 0;
		return speed;
	}
	pid.error = error;

	integral += error;
	if(integral > max){
		integral = max;
	}else if(integral < -max){
		integral = -max;
	}
	pid.integral = integral;

	pid.derivate = error-previous_error;


	speed = Kp*pid.error + Ki*pid.integral + Kd*pid.derivate;

	return speed;
}

