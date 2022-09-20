/**
 * @file motor_pwm.h
 * @brief Generate PWM signals for motors using TCC0 and TCC1
 * TCC0 has 6 CC regs
 * TCC1 has 4 CC regs
 * 
 * Thruster 1: DIO 13 = PA22 = TCC0[2]
 * Thruster 2: DIO 12 = PA23 = TCC0[3]
 * Thruster 3: DIO 11 = PA21 = TCC0[1]
 * Thruster 4: DIO 10 = PA20 = TCC0[0]
 * Thruster 5: DIO  9 = PA19 = TCC1[3]
 * Thruster 6: DIO  7 = PA18 = TCC1[2]
 * Thruster 7: DIO  1 = PA17 = TCC1[1]
 * Thruster 8: DIO  0 = PA16 = TCC1[0]
 * 
 * ESCs use 1500usec pulse for 0%, 1100us for -100% and 1900us for 100%
 * As such, timers are configured to generate a pulse with a 1901usec period
 * The high time is varied from 1100usec to 1900usec to set thruster speeds
 * 
 * TCC0 and TCC1 clocks derived from GCLK1 = 48MHz
 * Divider of 16 is configured: 48MHz / 16 = 3MHz timer clock
 * Therefore there are 3 counts per microsecond (usec)
 * 
 * @author Marcus Behel
 * @author Chima Nwosu
 */

#pragma once


/**
 * Initialize TCC0 and TCC1 for motor pwm signal generation
 */
void motor_pwm_init(void);

/**
 * Set motor speeds for all thrusters (updates pwm waves)
 * @param speeds Array of speeds for all 8 trhusters (thruster # = index + 1)
 */
void motor_pwm_set(float *speeds);
