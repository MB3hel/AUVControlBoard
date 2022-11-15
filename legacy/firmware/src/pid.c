/**
 * @file pid.c
 * @author Marcus Behel
 */

#include <pid.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pid_reset(pid_t *pid){
    pid->integral = 0;
    pid->last_error = 0;
}

float pid_get_output(pid_t *pid, float current_pv){
    #define MIN(a, b)       ((a < b) ? a : b)
    #define MAX(a, b)       ((a > b) ? a : b)

    // Start with feedforward gain
    float output = pid->kf * pid->setpoint;

    // Calculate current error
    float curr_err = pid->setpoint - current_pv;

    // Proportional gain
    output += pid->kp * curr_err;

    // Integral gain
    pid->integral += curr_err;
    output += pid->ki * pid->integral;

    // Derivative gain
    output += pid->kd * (curr_err - pid->last_error);
    pid->last_error = curr_err;

    // Limit output range
    return MAX(pid->min, MIN(output, pid->max));
}