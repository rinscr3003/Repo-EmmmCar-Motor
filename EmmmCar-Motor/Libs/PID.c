// lib from https://github.com/pms67/PID

#include "PID.h"

void PIDController_Init(PIDController *pid)
{

	/* Clear controller variables */
	pid->prevError = 0.0f;
	pid->prevError2 = 0.0f;
	pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement)
{

	/*
	 * Error signal
	 */
	float error = setpoint - measurement;

	/*
	 * Proportional
	 */
	float proportional = pid->Kp * (error - pid->prevError);

	/*
	 * Integral
	 */
	float integrator = pid->Ki * pid->T * error;

	/*
	 * Derivative
	 */

	float differentiator = pid->Kd * (error - 2.0f * pid->prevError + pid->prevError2) / pid->T;

	/*
	 * Compute output and apply limits
	 */
	pid->out += proportional + integrator + differentiator;

	if (pid->out > pid->limMax)
	{

		pid->out = pid->limMax;
	}
	else if (pid->out < pid->limMin)
	{

		pid->out = pid->limMin;
	}

	/* Store error and measurement for later use */
	pid->prevError2 = pid->prevError;
	pid->prevError = error;

	/* Return controller output */
	return pid->out;
}
