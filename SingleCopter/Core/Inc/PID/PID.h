#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#define SAMPLE_TIME_S 0.001f   // PID sample time = control loop period (1 kHz)

/* PID output and integral limits — compartidos entre main.c y usart.c */
#define PID_LIM_MIN_INT_ROLL  -50.0f
#define PID_LIM_MAX_INT_ROLL  +50.0f
#define PID_LIM_MIN_ROLL      -400.0f
#define PID_LIM_MAX_ROLL      +400.0f

#define PID_LIM_MIN_INT_PITCH PID_LIM_MIN_INT_ROLL
#define PID_LIM_MAX_INT_PITCH PID_LIM_MAX_INT_ROLL
#define PID_LIM_MIN_PITCH     PID_LIM_MIN_ROLL
#define PID_LIM_MAX_PITCH     PID_LIM_MAX_ROLL

#define PID_LIM_MIN_INT_YAW   -100.0f
#define PID_LIM_MAX_INT_YAW   +100.0f
#define PID_LIM_MIN_YAW       -400.0f
#define PID_LIM_MAX_YAW       +400.0f

#define CLIP(value, min, max) \
    ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))


// PID controller structure
typedef struct {
    float Kp;                  // Proportional gain
    float Ki;                  // Integral gain
    float Kd;                  // Derivative gain
    float delta_T;             // Sample time (in seconds)
    float prev_error;          // Previous error
    float prev_setpoint;	   //
    float integral;            // Integral term
    float prev_derivative;     // Derivative term
    float min_integral_limit;  // integral limits
    float max_integral_limit;  // integral limits
    float min_output_limit;	   // min PID limit
    float max_output_limit;	   // max PID limit
    float tau; 				   // Derivate filte
    float filter;              // Derivative filter coefficient
} PIDController;

void initializePID(PIDController *pid,
				   float Kp,
				   float Ki,
				   float Kd,
				   float T,
				   float min_integral_limit,
				   float max_integral_limit,
				   float min_output_limit,
				   float max_output_limit);

float updatePID(PIDController *pid, float setpoint, float measurement, float measurement_dot);

void resetPID(PIDController *pid);

#endif
