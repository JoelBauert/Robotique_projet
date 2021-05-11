/*
 * pid.h
 *
 *  Created on: 27 avr. 2021
 *      Author: jessica
 */

#ifndef PID_H_
#define PID_H_


void set_pid_param(float kp, float ki, float kd, float Threshold);
float calcul_pid(float val1, float val2, float max);

#endif /* PID_H_ */
