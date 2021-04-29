/*
 * pid.c
 *
 *  Created on: 27 avr. 2021
 *      Author: jessica
 */
#include <pid.h>
#include <math.h>

typedef struct
{
	float error;
	float integral;
	float derivate;
} PID_obj;

static float Kp, Ki, Kd, threshold;
/*
*	set the parameters needed to compute a pid regulator
*	params :				kp, ki, kd, threshold
*/
void set_pid_param(float kp, float ki, float kd, float Threshold)
{
	Kp = kp;
	Ki = ki;
	Kd = kd;
	threshold = Threshold;
}
/*
*	compute a pid regulator
*	params :
*	val1: value that we mesure
*	val2: value to reach
*	max:  limit for the integrator
*
*	output: output_regulated
*/
float calcul_pid(float val1, float val2, float max)
{
	PID_obj pid;
	float output_regulated;
	static float integral = 0;
	static float previous_error = 0;

	float error = val1-val2;

	if(fabs(error) <= threshold)
	{
		integral = 0;
		output_regulated = 0;
		return output_regulated;
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


	output_regulated = Kp*pid.error + Ki*pid.integral + Kd*pid.derivate;
	if(output_regulated > max){
		output_regulated = max;
		}else if(output_regulated < -max){
			output_regulated = -max;
		}
	return output_regulated;
}

