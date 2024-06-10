#include "PID/PID.h"


// Initialize the PID controller
void initializePID(PIDController *pid,
				   float Kp,
				   float Ki,
				   float Kd,
				   float T,
				   float min_integral_limit,
				   float max_integral_limit,
				   float min_output_limit,
				   float max_output_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->T = T;

    pid->tau = 1e-8f;
    pid->prev_setpoint = 0.0f;

    pid->prev_error      = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->integral        = 0.0f;

    pid->min_integral_limit = min_integral_limit;
    pid->max_integral_limit = max_integral_limit;

    pid->min_output_limit = min_output_limit;
    pid->max_output_limit = max_output_limit;
}

// Update the PID controller
float updatePID(PIDController *pid,
		        float setpoint,
				float measurement,
				float measurement_dot)
{
	float error = setpoint - measurement;
    // Proportional term
    float P = pid->Kp * error;
    // Integral term
    pid->integral = pid->integral + 0.5 * pid->T  * (error + pid->prev_error);
    // constrain integral output
    float I = CLIP(pid->Ki * pid->integral, pid->min_integral_limit, pid->max_integral_limit);
    // Derivative term
    float filter = 2*(pid->tau - pid->T) / 2*(pid->tau + pid->T);
    float D = 2 * pid->Kd * (((setpoint - pid->prev_setpoint) / pid->T) - measurement_dot) + filter*pid->prev_derivative;
    // Compute the control output, constrain output
    float output = CLIP(P + I + D, pid->min_output_limit, pid->max_output_limit);
    // Save the current error for the next iteration, e[k-1]
    pid->prev_error = error;
    // Save the current error for the next iteration,  D[k-1]
    pid->prev_derivative = D;

    return output;
}
