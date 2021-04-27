/*
 * pid.h
 *
 *  Created on: 27 avr. 2021
 *      Author: jessica
 */

#ifndef PID_H_
#define PID_H_
typedef struct
{
	float error;
	float integral;
	float derivate;
} PID_obj;

void set_pid_param(float kp, float ki, float kd);
float calcul_pid(float val1, float val2, float threshold, float max);


#endif /* PID_H_ */
