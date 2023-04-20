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
#include <simulator.h>


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
    (void)timer;
    
    xSemaphoreTake(motor_mutex, portMAX_DELAY);
    motors_killed = true;
    if(sim_hijacked){
        for(unsigned int i = 0; i < 8; ++i){
            sim_speeds[i] = 0.0f;
        }
    }else{
        thruster_set((float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    }
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

// Rotate a vector (x, y, z) buy a quaternion q
static inline void rotate_vector(float *dx, float *dy, float *dz, float sx, float sy, float sz, quaternion_t *q){
    quaternion_t qv;
    qv.w = 0.0f;
    qv.x = sx;
    qv.y = sy;
    qv.z = sz;    
    
    quaternion_t qconj;
    quat_conjugate(&qconj, q);

    quaternion_t qr;
    quat_multiply(&qr, &qv, &qconj);
    quat_multiply(&qr, q, &qr);
    *dx = qr.x;
    *dy = qr.y;
    *dz = qr.z;
}

// Minimum rotation from a to b (as quaternion)
static inline void quat_diff(quaternion_t *dest, quaternion_t *a, quaternion_t *b){
    float dot;
    quat_dot(&dot, a, b);
    quaternion_t b_inv;
    quat_inverse(&b_inv, b);
    if(dot < 0.0){
        quat_multiply_scalar(&b_inv, &b_inv, -1.0f);
    }
    quat_multiply(dest, a, &b_inv);
}

// Quaternion rotation from vector a to vector b
static inline void quat_between(quaternion_t *dest, float ax, float ay, float az, float bx, float by, float bz){
    float dot = ax*bx + ay*by + az*bz;
    float a_len2 = ax*ax + ay*ay + az*az;
    float b_len2 = bx*bx + by*by + bz*bz;
    float p = sqrtf(a_len2 * b_len2);
    float cross_x = ay*bz - az*by;
    float cross_y = az*bx - ax*bz;
    float cross_z = ax*by - ay*bx;
    if(dot / p == -1){
        // 180 degree rotation
        dest->w = 0.0f;
        dest->x = cross_x;
        dest->y = cross_y;
        dest->z = cross_z;
    }else{
        dest->w = dot + p;
        dest->x = cross_x;
        dest->y = cross_y;
        dest->z = cross_z;
    }
    quat_normalize(dest, dest);
}

// Twist part of swing-twist decomposition
// calculates twist of q around axis described by vector d = <d_x, d_y, d_z>
// Note that d must be normalized!!!
static inline void quat_twist(quaternion_t *twist, quaternion_t *q, float d_x, float d_y, float d_z){
    // r = axis of rotation = <q.x, q.y, q.z>
    float r_x = q->x;
    float r_y = q->y;
    float r_z = q->z;

    // dot = <r, d> (dot product)
    float dot = r_x*d_x + r_y*d_y + r_z*d_z;

    // p = projection or r on d = ((<r, d>) / (|d|^2))d
    // p = <r,d>d if d is unit vector (as it is assumed to be here)
    float p_x = dot * d_x;
    float p_y = dot * d_y;
    float p_z = dot * d_z;

    // twist = (w=q.w, x=p.x, y=p.y, z=p.z).normalized()
    twist->w = q->w;
    twist->x = p_x;
    twist->y = p_y;
    twist->z = p_z;
    quat_normalize(twist, twist);

    // However, if <r, d> is negative, the twist will be opposite the axis of rotation
    // Thus, negate the quaternion so twist's axis points in same direction as d
    // and it's angle will be the expected sign
    if(dot < 0.0f){
        quat_multiply_scalar(twist, twist, -1.0f);
    }
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
        if(sim_hijacked){
            for(unsigned int i = 0; i < 8; ++i){
                sim_speeds[i] = speeds[i];
            }
        }else{
            thruster_set(speeds);
        }
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
    float grav_x = 2.0f * (-curr_quat.x*curr_quat.z + curr_quat.w*curr_quat.y);
    float grav_y = 2.0f * (-curr_quat.w*curr_quat.x - curr_quat.y*curr_quat.z);
    float grav_z = -curr_quat.w*curr_quat.w + curr_quat.x*curr_quat.x + curr_quat.y*curr_quat.y - curr_quat.z*curr_quat.z;
    float grav_mag = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
    grav_x /= grav_mag;
    grav_y /= grav_mag;
    grav_z /= grav_mag;
    quaternion_t qrot;
    quat_between(&qrot, grav_x, grav_y, grav_z, 0.0f, 0.0f, -1.0f);
    quat_inverse(&qrot, &qrot);
    rotate_vector(&x, &y, &z, x, y, z, &qrot);
    rotate_vector(&pitch, &roll, &yaw, pitch, roll, yaw, &qrot);
    mc_set_local(x, y, z, pitch, roll, yaw);
}

void mc_sassist_tune_pitch(float kp, float ki, float kd, float limit, bool invert){
    pitch_pid.kP = kp;
    pitch_pid.kI = ki;
    pitch_pid.kD = kd;
    pitch_pid.max = limit;
    pitch_pid.min = -limit;
    pitch_pid.invert = invert;
}

void mc_sassist_tune_roll(float kp, float ki, float kd, float limit, bool invert){
    roll_pid.kP = kp;
    roll_pid.kI = ki;
    roll_pid.kD = kd;
    roll_pid.max = limit;
    roll_pid.min = -limit;
    roll_pid.invert = invert;
}

void mc_sassist_tune_yaw(float kp, float ki, float kd, float limit, bool invert){
    yaw_pid.kP = kp;
    yaw_pid.kI = ki;
    yaw_pid.kD = kd;
    yaw_pid.max = limit;
    yaw_pid.min = -limit;
    yaw_pid.invert = invert;
}

void mc_sassist_tune_depth(float kp, float ki, float kd, float limit, bool invert){
    depth_pid.kP = kp;
    depth_pid.kI = ki;
    depth_pid.kD = kd;
    depth_pid.max = limit;
    depth_pid.min = -limit;
    depth_pid.invert = invert;
}

void mc_set_sassist(float x, float y, float yaw_spd,
        euler_t target_euler,
        float target_depth,
        quaternion_t curr_quat,
        float curr_depth,
        bool yaw_target){

    float base_xrot = 0.0f;
    float base_yrot = 0.0f;
    float base_zrot = 0.0f;

    if(!yaw_target){
        // euler_t curr_euler;
        // quat_to_euler(&curr_euler, &curr_quat);
        // euler_rad2deg(&curr_euler, &curr_euler);
        // target_euler.yaw = curr_euler.yaw;

        quaternion_t curr_twist;
        quat_twist(&curr_twist, &curr_quat, 0, 0, 1);
        float curr_yaw = atan2f(-2.0f * (curr_twist.x*curr_twist.y - curr_twist.w*curr_twist.z), 1.0f - 2.0f * (curr_twist.x*curr_twist.x + curr_twist.z*curr_twist.z));
        curr_yaw *= 180.0f / M_PI;

        target_euler.yaw = curr_yaw;


        quaternion_t curr_swing;
        quaternion_t curr_twist_conj;
        quat_conjugate(&curr_twist_conj, &curr_twist);
        quat_multiply(&curr_swing, &curr_quat, &curr_twist_conj);

        // Same transform as global mode translation
        float grav_x = 2.0f * (-curr_quat.x*curr_quat.z + curr_quat.w*curr_quat.y);
        float grav_y = 2.0f * (-curr_quat.w*curr_quat.x - curr_quat.y*curr_quat.z);
        float grav_z = -curr_quat.w*curr_quat.w + curr_quat.x*curr_quat.x + curr_quat.y*curr_quat.y - curr_quat.z*curr_quat.z;
        float grav_mag = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
        grav_x /= grav_mag;
        grav_y /= grav_mag;
        grav_z /= grav_mag;
        quaternion_t qrot;
        quat_between(&qrot, grav_x, grav_y, grav_z, 0.0f, 0.0f, -1.0f);
        quat_inverse(&qrot, &qrot);
        rotate_vector(&base_xrot, &base_yrot, &base_zrot, 0.0f, 0.0f, yaw_spd, &qrot);
    }

    // Convert target to quaternion
    quaternion_t target_quat;
    euler_to_quat(&target_quat, &target_euler);


    // TODO: Test this. Dev math.
    float dot;
    quat_dot(&dot, &curr_quat, &target_quat);

    quaternion_t q_c_conj;
    if(dot < 0.0f){
        quaternion_t temp;
        temp.w = curr_quat.w;
        temp.x = curr_quat.x;
        temp.y = curr_quat.y;
        temp.z = curr_quat.z;
        quat_multiply_scalar(&temp, &temp, -1.0f);
        quat_conjugate(&q_c_conj, &temp);
    }else{
        quat_conjugate(&q_c_conj, &curr_quat);
    }

    quaternion_t q_d;
    quat_multiply(&q_d, &q_c_conj, &target_quat);

    float mag = sqrtf(q_d.x*q_d.x + q_d.y*q_d.y + q_d.z*q_d.z);
    float theta = 2.0f * atan2f(mag, q_d.w);
    float ax = q_d.x;
    float ay = q_d.y;
    float az = q_d.z;
    if(mag > 0.001f){
        ax /= mag;
        ay /= mag;
        az /= mag;
    }else{
        // TODO: Maybe? Probably better to not?
        // ax = 0.0f;
        // ay = 0.0f;
        // az = 0.0f;
    }

    // TODO: This feels like it won't quite work in 3D
    // maybe it does, but need to work out the math fully
    // |theta| > 180.0 deg means we're trying to rotate the long way around
    // This is not ideal.
    // So fix it.
    // if(theta > (float)M_PI){
    //     theta -= M_PI;
    //     theta -= M_PI;
    // }
    // if(theta < -((float)M_PI)){
    //     theta += M_PI;
    //     theta += M_PI;
    // }

    float ww_x = ax * theta;
    float ww_y = ay * theta;
    float ww_z = az * theta;

    quaternion_t q_ww;
    q_ww.w = 0.0f;
    q_ww.x = ww_x;
    q_ww.y = ww_y;
    q_ww.z = ww_z;

    // quaternion_t q_wv;
    // quat_multiply(&q_wv, &curr_quat, &q_ww);

    // float wv_x = q_wv.x;
    // float wv_y = q_wv.y;
    // float wv_z = q_wv.z;

    float wv_x = ww_x;
    float wv_y = ww_y;
    float wv_z = ww_z;

    // Use PID controllers to calculate current outputs
    float z = -pid_calculate(&depth_pid, curr_depth - target_depth);
    float pitch = pid_calculate(&pitch_pid, wv_x);
    float roll = pid_calculate(&roll_pid, wv_y);
    float yaw = pid_calculate(&yaw_pid, wv_z);

    // float pitch = 0.0f;
    // float roll = 0.0f;
    // float yaw = 0.0f;

    if(!yaw_target){
        // Need to add PID outputs to base speeds of vehicle
        // Base speeds came from rotation of yaw speed onto vehicle basis
        pitch += base_xrot;
        roll += base_yrot;
        yaw += base_zrot;

        // Proportionally scale speeds so all have magnitude less than 1.0
        float maxmag = 1.0f;
        float abspitch = fabsf(pitch);
        if(abspitch > maxmag){
            maxmag = abspitch;
        }
        float absroll = fabsf(roll);
        if(absroll > maxmag){
            maxmag = absroll;
        }
        float absyaw = fabsf(yaw);
        if(absyaw > maxmag){
            maxmag = absyaw;
        }
        pitch /= maxmag;
        roll /= maxmag;
        yaw /= maxmag;
    }

    // Apply same rotation as in GLOBAL mode to translation target
    float grav_x = 2.0f * (-curr_quat.x*curr_quat.z + curr_quat.w*curr_quat.y);
    float grav_y = 2.0f * (-curr_quat.w*curr_quat.x - curr_quat.y*curr_quat.z);
    float grav_z = -curr_quat.w*curr_quat.w + curr_quat.x*curr_quat.x + curr_quat.y*curr_quat.y - curr_quat.z*curr_quat.z;
    float grav_mag = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
    grav_x /= grav_mag;
    grav_y /= grav_mag;
    grav_z /= grav_mag;
    quaternion_t qrot;
    quat_between(&qrot, grav_x, grav_y, grav_z, 0.0f, 0.0f, -1.0f);
    quat_inverse(&qrot, &qrot);
    rotate_vector(&x, &y, &z, x, y, z, &qrot);

    // Target motion now relative to the robot's axes
    mc_set_local(x, y, z, pitch, roll, yaw);
}

void mc_set_sassist_old(float x, float y, float yaw,
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

void mc_set_dhold(float x, float y, float pitch, float roll, float yaw, float target_depth, quaternion_t curr_quat, float curr_depth){
    float z = pid_calculate(&depth_pid, target_depth - curr_depth);
    mc_set_global(x, y, z, pitch, roll, yaw, curr_quat);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

