/**
 * Floating point based PID controller implementation
 * 
 * @file pid.h
 * @author Marcus Behel
 */

#pragma once

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Typedefs
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct{
    // Intended to be directly modified by user
    float kp, ki, kd, kf;                       // Gains (including feed-forward gain)
    float min, max;                             // Values to limit PID output to
    float setpoint;                             // Current setpoint
    
    // NOT intended to be directly modified by user
    float integral;                             // Current accumulated (integral) value
    float last_error;                           // Previous error used in derivative value calculations
} pid_t;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// For now, just an alias for reset function
// Kept this way in case init requires different behavior later
#define pid_init            pid_reset

/**
 * Reset PID controller to initial state
 * @param pid PID to reset
 */
void pid_reset(pid_t *pid);

/**
 * Calculate the current output of the PID controller
 * 
 * MUST CONFIGURE GAINS, MIN/MAX, AND SETPOINT BEFORE CALLING THIS FUNCTION
 * 
 * Note: This does NOT use a dt scale factor, thust it is important to call at 
 * fairly regular intervals to ensure stability
 * 
 * @param pid PID to work on
 * @param current_pv Current value of the process variable
 */
float pid_get_output(pid_t *pid, float current_pv);
