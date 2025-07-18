
#include "PID/PID.h"

/**
 * @brief Initializes the PID controller structure with specified parameters.
 *
 * This function sets the proportional (Kp), integral (Ki), and derivative (Kd) gains,
 * the controller sample time (T), and the limits for the integral and output terms.
 * It also initializes internal state variables such as previous error, previous derivative,
 * and integral accumulator to zero, and sets a default value for the derivative filter time constant (tau).
 *
 * @param pid Pointer to the PIDController structure to initialize.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param delta_T Sample time (in seconds).
 * @param min_integral_limit Minimum allowable value for the integral term (anti-windup).
 * @param max_integral_limit Maximum allowable value for the integral term (anti-windup).
 * @param min_output_limit Minimum allowable output value.
 * @param max_output_limit Maximum allowable output value.
 */

void initializePID(PIDController *pid,
				   float Kp,
				   float Ki,
				   float Kd,
				   float delta_T,
				   float min_integral_limit,
				   float max_integral_limit,
				   float min_output_limit,
				   float max_output_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->delta_T = delta_T;
    pid->tau = 0.0005f;
    pid->filter = (2 * pid->tau - pid->delta_T) / (2 * pid->tau + pid->delta_T);
    
    // Initialize internal state variables
    pid->prev_setpoint = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->integral        = 0.0f;
    // Set limits for integral and output
    pid->min_integral_limit = min_integral_limit;
    pid->max_integral_limit = max_integral_limit;
    pid->min_output_limit = min_output_limit;
    pid->max_output_limit = max_output_limit;
}

/**
 * @brief Updates the PID controller with the current setpoint and measurement.
 *
 * This function calculates the control output based on the PID algorithm using the provided setpoint,
 * measurement, and measurement rate of change (derivative). It computes the proportional, integral,
 * and derivative terms, applies anti-windup to the integral term, and constrains the output within specified limits.
 *
 * @param pid Pointer to the PIDController structure.
 * @param setpoint Desired target value for the controlled variable.
 * @param measurement Current measured value of the controlled variable.
 * @param measurement_dot Rate of change (derivative) of the measurement, typically representing the measured velocity or angular rate;
 *                        this value is used in the derivative term to improve noise rejection and avoid derivative kick by differentiating 
 *                        the measurement instead of the error.
 * @return The computed control output, constrained within specified limits.
 */
float updatePID(PIDController *pid,
		        float setpoint,
				float measurement,
				float measurement_dot)
{
	float error = setpoint - measurement;
    // Proportional term
    float P = pid->Kp * error;
    // Integral term (using trapezoidal integration for improved accuracy)
    pid->integral = pid->integral + 0.5 * pid->delta_T  * (error + pid->prev_error);
    // constrain internal integral accumulator to prevent windup
    pid->integral = CLIP(pid->integral, pid->min_integral_limit, pid->max_integral_limit);
    float I = pid->Ki * pid->integral;
    // Derivative term
    //float D = 2 * pid->Kd * (((setpoint - pid->prev_setpoint) / pid->delta_T) - measurement_dot) + pid->filter*pid->prev_derivative;
    float D = pid->Kd * (((setpoint - pid->prev_setpoint) / pid->delta_T) - measurement_dot);
    // Compute the control output, constrain output
    float output = CLIP(P + I + D, pid->min_output_limit, pid->max_output_limit);
    // Save the current error for the next iteration, e[k-1]
    pid->prev_error = error;
    // Save the current error for the next iteration,  D[k-1]
    pid->prev_derivative = D;
    // Save the current setpoint for the next iteration
    pid->prev_setpoint = setpoint;

    return output;
}

/**
 * @brief Resets the internal state of the given PID controller.
 *
 * This function sets the integral, previous error, previous derivative,
 * and previous setpoint values of the PIDController structure to zero.
 * It is typically used to reinitialize the controller before starting
 * a new control loop or after a significant change in system state.
 *
 * @param pid Pointer to the PIDController structure to reset.
 */
/**
 * @brief Resets the internal state of the given PID controller.
 *
 * This function sets the integral, previous error, previous derivative,
 * and previous setpoint values of the PIDController structure to zero.
 * It is typically used to reinitialize the controller before starting
 * a new control loop or after a significant change in system state.
 *
 * @param pid Pointer to the PIDController structure to reset.
 */
void resetPID(PIDController *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->prev_setpoint = 0.0f;
}
