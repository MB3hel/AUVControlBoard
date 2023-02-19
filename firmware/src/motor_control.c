/*
 * Copyright 2022 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#include <motor_control.h>
#include <app.h>
#include <matrix.h>
#include <thruster.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <cmdctrl.h>
#include <pid.h>
#include <math.h>
#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MOTOR_WDOG_PERIOD_MS            1500

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool mc_invert[8];                                      // Tracks thruster inversions

static float dof_matrix_arr[8*6];                       // Backing array for DoF matrix
static matrix dof_matrix;                               // DoF matrix

static float overlap_arrs[8][8];                        // Backing arrays for overlap vectors
static matrix overlap_vectors[8];                       // overlaps vectors

static bool motors_killed;                              // Motor (watchdog) state
static TimerHandle_t motor_wdog_timer;                  // Timer to implement motor watchdog

static SemaphoreHandle_t motor_mutex;                   // Ensures motor & watchdog access is thread safe

// PID controllers for SASSIST mode
static pid_controller_t pitch_pid, roll_pid, yaw_pid, depth_pid;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Initialization & Setup
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void mc_wdog_timeout(TimerHandle_t timer);

void mc_init(void){
    // Initialize matrices
    matrix_init_static(&dof_matrix, dof_matrix_arr, 8, 6);
    for(unsigned int i = 0; i < 8; ++i)
        matrix_init_static(&overlap_vectors[i], overlap_arrs[i], 8, 1);

    // Initialize pid controllers
    pitch_pid.kP = 0.0f;
    pitch_pid.kI = 0.0f;
    pitch_pid.kD = 0.0f;
    pitch_pid.min = -1.0f;
    pitch_pid.max = 1.0f;
    PID_RESET(pitch_pid);
    roll_pid.kP = 0.0f;
    roll_pid.kI = 0.0f;
    roll_pid.kD = 0.0f;
    roll_pid.min = -1.0f;
    roll_pid.max = 1.0f;
    PID_RESET(roll_pid);
    yaw_pid.kP = 0.0f;
    yaw_pid.kI = 0.0f;
    yaw_pid.kD = 0.0f;
    yaw_pid.min = -1.0f;
    yaw_pid.max = 1.0f;
    PID_RESET(yaw_pid);
    depth_pid.kP = 0.0f;
    depth_pid.kI = 0.0f;
    depth_pid.kD = 0.0f;
    depth_pid.min = -1.0f;
    depth_pid.max = 1.0f;
    PID_RESET(depth_pid);

    // Create required RTOS objects
    motor_mutex = xSemaphoreCreateMutex();
    motor_wdog_timer = xTimerCreate(
        "mwdog_tim",
        pdMS_TO_TICKS(MOTOR_WDOG_PERIOD_MS),
        pdFALSE,                                        // No autoreload = singleshot = becomes inactive after timeout
        NULL,
        mc_wdog_timeout
    );

    // Motors killed at startup
    motors_killed = true;

    // Default all motors to non-inverted
    for(unsigned int i = 0; i < 8; ++i){
        mc_invert[i] = false;
    }
}

void mc_set_dof_matrix(unsigned int thruster_num, float *row_data){
    matrix_set_row(&dof_matrix, thruster_num - 1, row_data);
}

void mc_recalc(void){
    // Called when done updating DoF matrix

    // Construct contribution  matrix (used to calculate overlap vectors)
    float contribution_arr[8*6];
    matrix contribution_matrix;
    matrix_init_static(&contribution_matrix, contribution_arr, 8, 6);
    for(size_t row = 0; row < contribution_matrix.rows; ++row){
        for(size_t col = 0; col < contribution_matrix.cols; ++col){
            float dof_item;
            matrix_get_item(&dof_item, &dof_matrix, row, col);
            matrix_set_item(&contribution_matrix, row, col, (dof_item != 0) ? 1 : 0);
        }
    }

    // Construct overlap vectors
    float rowdata[8];
    float v_arr[6];
    matrix v;
    matrix_init_static(&v, v_arr, 6, 1);
    for(size_t r = 0; r < contribution_matrix.rows; ++r){
        matrix_init_static(&overlap_vectors[r], overlap_arrs[r], contribution_matrix.rows, 1);

        // v = contribution_matrix row r transposed
        matrix_get_row(rowdata, &contribution_matrix, r);
        matrix_set_col(&v, 0, rowdata);

        matrix_mul(&overlap_vectors[r], &contribution_matrix, &v);

        for(size_t row = 0; row < overlap_vectors[r].rows; ++row){
            for(size_t col = 0; col < overlap_vectors[r].cols; ++col){
                float item;
                matrix_get_item(&item, &overlap_vectors[r], row, col);
                matrix_set_item(&overlap_vectors[r], row, col, (item != 0) ? 1 : 0);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Motor Watchdog
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void mc_wdog_timeout(TimerHandle_t timer){
    xSemaphoreTake(motor_mutex, portMAX_DELAY);
    motors_killed = true;
    thruster_set((float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    cmdctrl_mwdog_change(false);
    xSemaphoreGive(motor_mutex);
}

bool mc_wdog_feed(void){
    bool ret;
    xSemaphoreTake(motor_mutex, portMAX_DELAY);
    // If timer is inactive, this starts the timer
    // If timer is active, this resets the timer
    xTimerReset(motor_wdog_timer, portMAX_DELAY);
    ret = motors_killed;
    if(motors_killed)
        cmdctrl_mwdog_change(true);
    motors_killed = false;
    xSemaphoreGive(motor_mutex);
    return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Motor control
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Skew symmetric matrix from a vector
 * 
 * @param outmat Matrix to store output in (skew symmetric matrix 3x3)
 * @param invec Vector (3x1) used to construct matrix
 * @return MAT_ERR_x code 
 */
static int skew3(matrix *outmat, matrix *invec){
    if(outmat->rows != 3 || outmat->cols != 3)
        return MAT_ERR_SIZE;
    float v[3];
    if(invec->rows == 1){
        if(invec->cols != 3)
            return MAT_ERR_SIZE;
        matrix_get_row(&v[0], invec, 0);
    }else if(invec->cols == 1){
        if(invec->rows != 3)
            return MAT_ERR_SIZE;
        matrix_get_col(&v[0], invec, 0);
    }else{
        return MAT_ERR_SIZE;
    }
    matrix_set_row(outmat, 0, (float[]){0, -v[2], v[1]});
    matrix_set_row(outmat, 1, (float[]){v[2], 0, -v[0]});
    matrix_set_row(outmat, 2, (float[]){-v[1], v[0], 0});
    return MAT_ERR_NONE;
}


void mc_set_raw(float *speeds){
    xSemaphoreTake(motor_mutex, portMAX_DELAY);   

    // Don't allow speed set while motors are killed
    if(!motors_killed){
        // Apply thruster inversions
        for(unsigned int i = 0; i < 8; ++i){
            if(mc_invert[i])
                speeds[i] *= -1;
        }

        // Actually set thruster speeds
        thruster_set(speeds);
    }

    xSemaphoreGive(motor_mutex);
}

void mc_set_local(float x, float y, float z, float pitch, float roll, float yaw){
    float target_arr[6];
    matrix target;
    matrix_init_static(&target, target_arr, 6, 1);
    matrix_set_col(&target, 0, (float[]){x, y, z, pitch, roll, yaw});

    // Limit input speeds to correct range
    for(size_t i = 0; i < 6; ++i){
        if(target_arr[i] > 1.0)
            target_arr[i] = 1.0;
        if(target_arr[i] < -1.0)
            target_arr[i] = -1.0;
    }

    float speed_arr[8];
    matrix speed_vec;
    matrix_init_static(&speed_vec, speed_arr, 8, 1);

    // Base speed calculation
    matrix_mul(&speed_vec, &dof_matrix, &target);

    // Scale motor speeds down as needed
    while(true){
        size_t idxrow, idxcol;
        float mval;
        matrix_absmax(&mval, &idxrow, &idxcol, &speed_vec);
        if(mval <= 1)
            break;
        for(size_t i = 0; i < overlap_vectors[idxrow].rows; ++i){
            float cval;
            matrix_get_item(&cval, &overlap_vectors[idxrow], i, 0);
            if(cval == 1){
                matrix_get_item(&cval, &speed_vec, i, 0);
                cval /= mval;
                matrix_set_item(&speed_vec, i, 0, cval);
            }
        }
    }    

    // Speed array already contains motor speeds in order
    // Because dof matrix rows are in order
    mc_set_raw(speed_arr);
}

void mc_set_global(float x, float y, float z, float pitch, float roll, float yaw, quaternion_t curr_quat){
    // Construct target motion vector
    float target_arr[6];
    matrix target;
    matrix_init_static(&target, target_arr, 6, 1);
    matrix_set_col(&target, 0, (float[]){x, y, z, pitch, roll, yaw});

    // Construct current gravity vector from quaternion
    float gvec_arr[3];
    matrix gravity_vector;
    matrix_init_static(&gravity_vector, gvec_arr, 3, 1);
    matrix_set_item(&gravity_vector, 0, 0, 2.0f * (-curr_quat.x*curr_quat.z + curr_quat.w*curr_quat.y));
    matrix_set_item(&gravity_vector, 1, 0, 2.0f * (-curr_quat.w*curr_quat.x - curr_quat.y*curr_quat.z));
    matrix_set_item(&gravity_vector, 2, 0, -curr_quat.w*curr_quat.w + curr_quat.x*curr_quat.x + curr_quat.y*curr_quat.y - curr_quat.z*curr_quat.z);

    // b is unit gravity vector
    float b_arr[3];
    float gravl2norm;
    matrix b;
    matrix_init_static(&b, b_arr, 3, 1);
    matrix_l2vnorm(&gravl2norm, &gravity_vector);
    if(gravl2norm < 0.1f)
        return; // Invalid gravity vector (norm should be non-zero)
    matrix_sc_div(&b, &gravity_vector, gravl2norm);

    // Expected unit gravity vector when "level"
    float a_arr[3];
    matrix a;
    matrix_init_static(&a, a_arr, 3, 1);
    matrix_set_col(&a, 0, (float[]){0, 0, -1});

    float v_arr[3];
    matrix v;
    matrix_init_static(&v, v_arr, 3, 1);
    matrix_vcross(&v, &a, &b);

    float c;
    matrix_vdot(&c, &a, &b);

    float sk_arr[9];
    matrix sk;
    matrix_init_static(&sk, sk_arr, 3, 3);
    skew3(&sk, &v);

    float I_arr[9];
    matrix I;
    matrix_init_static(&I, I_arr, 3, 3);
    matrix_ident(&I);

    float R_arr[9];
    matrix R;
    matrix_init_static(&R, R_arr, 3, 3);
    matrix_mul(&R, &sk, &sk);
    matrix_sc_div(&R, &R, 1 + c);
    matrix_add(&R, &R, &sk);
    matrix_add(&R, &R, &I);

    float tmp[6];
    float tltarget_arr[3], rltarget_arr[3], tgtarget_arr[3], rgtarget_arr[3];
    matrix tltarget, rltarget, tgtarget, rgtarget;
    matrix_init_static(&tltarget, tltarget_arr, 3, 1);
    matrix_init_static(&rltarget, rltarget_arr, 3, 1);
    matrix_init_static(&tgtarget, tgtarget_arr, 3, 1);
    matrix_init_static(&rgtarget, rgtarget_arr, 3, 1);
    matrix_get_col(&tmp[0], &target, 0);
    matrix_set_col(&tgtarget, 0, &tmp[0]);
    matrix_set_col(&rgtarget, 0, &tmp[3]);

    matrix_mul(&tltarget, &R, &tgtarget);
    matrix_mul(&rltarget, &R, &rgtarget);

    matrix_get_col(&tmp[0], &tltarget, 0);
    matrix_get_col(&tmp[3], &rltarget, 0);

    matrix_set_col(&target, 0, &tmp[0]);

    // Target is now a local target (stored in order in target_arr)
    mc_set_local(target_arr[0], target_arr[1], target_arr[2], target_arr[3], target_arr[4], target_arr[5]);
}

void mc_sassist_tune_pitch(float kp, float ki, float kd, float limit){
    pitch_pid.kP = kp;
    pitch_pid.kI = ki;
    pitch_pid.kD = kd;
    pitch_pid.max = limit;
    pitch_pid.min = -limit;
}

void mc_sassist_tune_roll(float kp, float ki, float kd, float limit){
    roll_pid.kP = kp;
    roll_pid.kI = ki;
    roll_pid.kD = kd;
    roll_pid.max = limit;
    roll_pid.min = -limit;
}

void mc_sassist_tune_yaw(float kp, float ki, float kd, float limit){
    yaw_pid.kP = kp;
    yaw_pid.kI = ki;
    yaw_pid.kD = kd;
    yaw_pid.max = limit;
    yaw_pid.min = -limit;
}

void mc_sassist_tune_depth(float kp, float ki, float kd, float limit){
    depth_pid.kP = kp;
    depth_pid.kI = ki;
    depth_pid.kD = kd;
    depth_pid.max = limit;
    depth_pid.min = -limit;
}

void mc_set_sassist(float x, float y, float yaw,
        euler_t target_euler,
        float target_depth,
        quaternion_t curr_quat,
        float curr_depth,
        bool use_yaw_pid){
    // ********************************************************************************** //
    // * WARNING: THIS IS A "TEMPORARY" METHOD OF DOING THIS USING EULER ANGLES         * //
    // * THIS WILL NOT WORK IN GIMBAL LOCK SCENARIOS!!!                                 * //
    // * THIS IS A STOPGAP TO ALLOW NORMAL USE CASES WHERE PITCH AND ROLL ARE           * //
    // * NEAR ZERO!!!                                                                   * //
    // * DO NOT ATTEMPT TO ROLL NEAR +/-90 IT WILL BREAK!!!                             * //
    // ********************************************************************************** //

    // Convert current orientation into euler angles
    euler_t curr_euler;
    quat_to_euler(&curr_euler, &curr_quat);
    euler_rad2deg(&curr_euler, &curr_euler);

    // Calculate PID outputs
    // TODO: Handle these as circular variables properly???
    //       This is hard with roll because -90 to +90 then it changes others...
    float z = pid_calculate(&depth_pid, target_depth - curr_depth);
    float pitch = pid_calculate(&pitch_pid, target_euler.pitch - curr_euler.pitch);
    float roll = pid_calculate(&roll_pid, target_euler.roll - curr_euler.roll);
    if(use_yaw_pid){
        yaw = pid_calculate(&yaw_pid, target_euler.yaw - curr_euler.yaw);
    }

    // Update speeds using GLOBAL mode math
    mc_set_global(x, y, z, pitch, roll, yaw, curr_quat);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

